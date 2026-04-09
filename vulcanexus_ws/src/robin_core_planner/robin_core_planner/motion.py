"""MoveIt motion planning wrapper for Pilz industrial motion planner."""

import time

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit.planning import MoveItPy, PlanRequestParameters


class MotionPlanner:
    """Wraps MoveItPy for Pilz PTP/LIN planning and execution.

    Attributes:
        robot: MoveItPy instance
        arm: PlanningComponent for the manipulator
        planning_scene_monitor: PlanningSceneMonitor
    """

    PLANNING_GROUP = "ur_manipulator"
    PILZ_PIPELINE = "pilz_industrial_motion_planner"
    PLANNER_PTP = "PTP"
    PLANNER_LIN = "LIN"

    def __init__(self):
        self.robot: MoveItPy | None = None
        self.arm = None
        self.planning_scene_monitor = None

    def initialize(self, logger) -> bool:
        """Create the MoveItPy instance. Call once after controller is ready."""
        try:
            self.robot = MoveItPy(node_name="robin_planner_moveit")
            self.arm = self.robot.get_planning_component(self.PLANNING_GROUP)
            self.planning_scene_monitor = self.robot.get_planning_scene_monitor()
            logger.info("MoveItPy instance created for UR10e")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize MoveItPy: {e}")
            return False

    @property
    def is_ready(self) -> bool:
        return self.robot is not None

    def get_current_pose(self, end_effector_link: str):
        """Read the current end-effector pose from the planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            return scene.current_state.get_pose(end_effector_link)

    def plan_and_execute(
        self,
        logger,
        sleep_time: float = 0.0,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
        planner_id: str | None = None,
        diagnostics: dict | None = None,
    ) -> bool:
        """Plan and execute the current goal state using Pilz.

        Assumes ``arm.set_start_state_to_current_state()`` and
        ``arm.set_goal_state(...)`` were called before this method.
        """
        planner = planner_id or self.PLANNER_PTP

        plan_params = PlanRequestParameters(self.robot, "")
        plan_params.planning_pipeline = self.PILZ_PIPELINE
        plan_params.planner_id = planner
        plan_params.max_velocity_scaling_factor = velocity_scaling
        plan_params.max_acceleration_scaling_factor = acceleration_scaling

        logger.info(
            f"Planning with Pilz {planner}: "
            f"vel_scale={velocity_scaling:.3f}, acc_scale={acceleration_scaling:.3f}")

        plan_result = self.arm.plan(single_plan_parameters=plan_params)

        if plan_result and plan_result.trajectory:
            try:
                duration_sec = plan_result.trajectory.get_duration()
                logger.info(f"Planned trajectory duration: {duration_sec:.2f}s (planner={planner})")
            except Exception:
                pass

            self.robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(sleep_time)
            return True

        logger.error("Planning failed!")
        if diagnostics:
            logger.error(
                "Planning diagnostics: "
                f"ee_link={diagnostics.get('ee_link', '-')}, "
                f"planner={planner}, vel={velocity_scaling:.3f}, acc={acceleration_scaling:.3f}, "
                f"target={diagnostics.get('target_pose', '-')}, "
                f"resolved={diagnostics.get('resolved_pose', '-')}, "
                f"current={diagnostics.get('current_pose', '-')}")
        return False

    def move_to_pose(
        self,
        target_pose: PoseStamped,
        end_effector_link: str,
        logger,
        resolve_tcp_fn=None,
        sleep_time: float = 0.0,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        planner_id: str | None = None,
        default_velocity_scaling: float = 0.3,
        default_acceleration_scaling: float = 0.3,
        move_label: str = "",
    ) -> bool:
        """Move end-effector to *target_pose* with optional TCP offset resolution.

        Args:
            target_pose: Desired goal for the active TCP frame.
            end_effector_link: MoveIt EE link name (wire_tip).
            logger: ROS logger.
            resolve_tcp_fn: Optional callable(PoseStamped)->PoseStamped for TCP offset.
            sleep_time: Seconds to sleep after execution.
            velocity_scaling / acceleration_scaling: Override defaults when provided.
            planner_id: 'PTP' or 'LIN'. Defaults to PTP.
            default_velocity_scaling / default_acceleration_scaling: Fallbacks.
        """
        resolved = resolve_tcp_fn(target_pose) if resolve_tcp_fn else target_pose

        target_diag = self._format_pose(target_pose)
        resolved_diag = self._format_pose(resolved)

        current_diag = "n/a"
        try:
            current_pose = self.get_current_pose(end_effector_link)
            current_diag = self._format_raw_pose(current_pose)
        except Exception:
            pass

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=resolved, pose_link=end_effector_link)

        vel = velocity_scaling if velocity_scaling is not None else default_velocity_scaling
        acc = acceleration_scaling if acceleration_scaling is not None else default_acceleration_scaling

        ok = self.plan_and_execute(
            logger,
            sleep_time,
            vel,
            acc,
            planner_id,
            diagnostics={
                "ee_link": end_effector_link,
                "target_pose": target_diag,
                "resolved_pose": resolved_diag,
                "current_pose": current_diag,
                "label": move_label,
            },
        )
        if not ok and move_label:
            logger.error(f"Planning step failed: {move_label}")
        return ok

    @staticmethod
    def _format_pose(pose_stamped: PoseStamped) -> str:
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        return (
            f"({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
            f"q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) "
            f"frame={pose_stamped.header.frame_id}"
        )

    @staticmethod
    def _format_raw_pose(pose) -> str:
        p = pose.position
        q = pose.orientation
        return (
            f"({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
            f"q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})"
        )

    def move_relative(
        self,
        dx: float, dy: float, dz: float,
        end_effector_link: str,
        base_frame: str,
        logger,
        resolve_tcp_fn=None,
        sleep_time: float = 0.0,
        default_velocity_scaling: float = 0.3,
        default_acceleration_scaling: float = 0.3,
    ) -> bool:
        """Move end-effector relative to current position."""
        import rclpy
        current = self.get_current_pose(end_effector_link)

        target = PoseStamped()
        target.header.frame_id = base_frame
        target.pose.position.x = current.position.x + dx
        target.pose.position.y = current.position.y + dy
        target.pose.position.z = current.position.z + dz
        target.pose.orientation = current.orientation

        return self.move_to_pose(
            target, end_effector_link, logger, resolve_tcp_fn,
            sleep_time,
            default_velocity_scaling=default_velocity_scaling,
            default_acceleration_scaling=default_acceleration_scaling,
        )

    @staticmethod
    def create_pose(
        x: float, y: float, z: float,
        base_frame: str,
        clock,
        orientation: Quaternion | None = None,
        fallback_orientation: Quaternion | None = None,
    ) -> PoseStamped:
        """Create a PoseStamped at the given position.

        Args:
            orientation: Explicit orientation. When *None*, *fallback_orientation*
                         (typically current EE orientation) is used.
        """
        pose = PoseStamped()
        pose.header.frame_id = base_frame
        pose.header.stamp = clock.now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = orientation if orientation is not None else fallback_orientation
        return pose
