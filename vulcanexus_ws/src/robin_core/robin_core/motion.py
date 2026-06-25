"""MoveIt motion planning wrapper for Pilz industrial motion planner."""

import math
import time
from dataclasses import dataclass, field

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import Constraints, JointConstraint


@dataclass
class PostureLockConfig:
    """Configuration for keeping plans within the current kinematic branch."""

    enabled: bool = False
    joint_names: tuple[str, ...] = field(default_factory=tuple)
    tolerances_rad: tuple[float, ...] = field(default_factory=tuple)


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
        self.arm: MoveItPy.PlanningComponent | None = None
        self.planning_scene_monitor: MoveItPy.PlanningSceneMonitor | None = None
        self._posture_lock = PostureLockConfig()

    def initialize(self, logger, node=None) -> bool:
        """Create the MoveItPy instance. Call once after controller is ready.

        Pass ``node`` when calling from a LifecycleNode. MoveItPy creates its
        own separate rclcpp node (``robin_planner_moveit``) which has no params;
        ``config_dict`` is the only way to forward robot_description_semantic
        (SRDF) and the other MoveIt settings to it.
        """
        if self.robot is not None:
            return True

        config_dict = None
        if node is not None:
            config_dict = self._collect_moveit_params(node, logger)
        try:
            self.robot = MoveItPy(
                node_name="robin_planner_moveit",
                config_dict=config_dict,
            )
            self.arm: MoveItPy.PlanningComponent = self.robot.get_planning_component(self.PLANNING_GROUP)
            self.planning_scene_monitor: MoveItPy.PlanningSceneMonitor = self.robot.get_planning_scene_monitor()
            logger.info("MoveItPy instance created for UR10e")
            return True
        except Exception as e:
            self.robot = None
            logger.error(f"Failed to initialize MoveItPy: {e}")
            return False

    @staticmethod
    def _collect_moveit_params(node, logger) -> dict | None:
        """Forward all node parameters to MoveItPy's config_dict."""
        params = {}
        try:
            result = node.list_parameters([], 0)
            for name in result.names:
                try:
                    params[name] = node.get_parameter(name).value
                except Exception:
                    logger.warn(f"Could not read parameter '{name}'")
        except Exception as e:
            logger.error(f"Failed to list node parameters: {e}")
            return None

        srdf = params.get("robot_description_semantic")
        if not srdf:
            logger.error(
                "robot_description_semantic (SRDF) is MISSING or empty on "
                f"node '{node.get_name()}'. MoveItPy will fail.")
            return params or None

        MotionPlanner._normalize_clock_qos_overrides(params, logger)
        logger.info(f"Forwarding {len(params)} parameters to MoveItPy")
        return params

    @staticmethod
    def _normalize_clock_qos_overrides(params: dict, logger) -> None:
        """Keep the in-process MoveItPy /clock QoS override internally consistent."""
        if not params.get("use_sim_time", False):
            return

        prefix = "qos_overrides./clock.subscription."
        defaults = {
            f"{prefix}history": "keep_last",
            f"{prefix}depth": 1,
            f"{prefix}reliability": "best_effort",
            f"{prefix}durability": "volatile",
        }

        changed = False
        for key, value in defaults.items():
            current = params.get(key)
            if current in (None, ""):
                params[key] = value
                changed = True

        if changed:
            logger.info("Normalized /clock QoS overrides for MoveItPy")

    def configure_posture_lock(
        self,
        *,
        enabled: bool,
        joint_names: list[str] | tuple[str, ...],
        tolerances_rad: list[float] | tuple[float, ...],
    ) -> None:
        """Configure posture-lock behaviour for all subsequent pose plans."""
        joint_names = tuple(str(name) for name in joint_names)
        tolerances = tuple(float(value) for value in tolerances_rad)
        if len(joint_names) != len(tolerances):
            raise ValueError(
                "posture_lock_joint_names and posture_lock_tolerances_rad "
                "must have the same length"
            )
        if enabled and not joint_names:
            raise ValueError(
                "posture_lock_joint_names must not be empty when posture lock "
                "is enabled"
            )

        if enabled and self.robot is not None and self.arm is not None:
            model = self.robot.get_robot_model()
            group = model.get_joint_model_group(self.PLANNING_GROUP)
            group_joint_names = set(group.active_joint_model_names)
            missing = [name for name in joint_names if name not in group_joint_names]
            if missing:
                raise ValueError(
                    "posture_lock_joint_names are not part of the planning "
                    f"group '{self.PLANNING_GROUP}': {missing}"
                )

        self._posture_lock = PostureLockConfig(
            enabled=bool(enabled),
            joint_names=joint_names,
            tolerances_rad=tolerances,
        )

    @staticmethod
    def _angle_distance(a: float, b: float) -> float:
        """Shortest signed angular distance from *b* to *a* in radians."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _get_current_group_joint_positions(self) -> dict[str, float]:
        """Return the current planning-group joint positions keyed by name."""
        model = self.robot.get_robot_model()
        group = model.get_joint_model_group(self.PLANNING_GROUP)
        joint_names = list(group.active_joint_model_names)

        with self.planning_scene_monitor.read_only() as scene:
            positions = scene.current_state.get_joint_group_positions(
                self.PLANNING_GROUP
            )

        return {
            name: float(position)
            for name, position in zip(joint_names, positions, strict=True)
        }

    @staticmethod
    def _build_joint_constraints(
        anchor_positions: dict[str, float],
        joint_names: tuple[str, ...],
        tolerances_rad: tuple[float, ...],
    ) -> Constraints:
        """Build joint path constraints anchored to the supplied positions."""
        constraints = Constraints()
        constraints.name = "posture_lock"

        for joint_name, tolerance in zip(joint_names, tolerances_rad, strict=True):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(anchor_positions[joint_name])
            joint_constraint.tolerance_above = float(tolerance)
            joint_constraint.tolerance_below = float(tolerance)
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    @staticmethod
    def _extract_joint_trajectory(trajectory):
        """Extract a JointTrajectory from a MoveItPy robot trajectory object."""
        if trajectory is None:
            return None
        if hasattr(trajectory, "joint_names") and hasattr(trajectory, "points"):
            return trajectory

        robot_traj_msg_fn = getattr(trajectory, "get_robot_trajectory_msg", None)
        if callable(robot_traj_msg_fn):
            try:
                return robot_traj_msg_fn().joint_trajectory
            except Exception:
                pass

        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        if hasattr(joint_trajectory, "joint_names") and hasattr(joint_trajectory, "points"):
            return joint_trajectory
        return None

    @classmethod
    def _trajectory_respects_posture_lock(
        cls,
        joint_trajectory,
        anchor_positions: dict[str, float],
        joint_names: tuple[str, ...],
        tolerances_rad: tuple[float, ...],
    ) -> tuple[bool, str]:
        """Check all trajectory points stay within the configured posture envelope."""
        if joint_trajectory is None:
            return False, "planned trajectory could not be inspected"
        if not joint_trajectory.points:
            return False, "planned trajectory contained no joint points"

        index_by_name = {
            name: index for index, name in enumerate(joint_trajectory.joint_names)
        }
        for joint_name in joint_names:
            if joint_name not in index_by_name:
                return False, f"trajectory is missing posture-lock joint '{joint_name}'"
            if joint_name not in anchor_positions:
                return False, f"missing anchor position for joint '{joint_name}'"

        for point_index, point in enumerate(joint_trajectory.points):
            for joint_name, tolerance in zip(
                joint_names, tolerances_rad, strict=True
            ):
                position = float(point.positions[index_by_name[joint_name]])
                error = abs(cls._angle_distance(position, anchor_positions[joint_name]))
                if error > tolerance:
                    return (
                        False,
                        f"joint '{joint_name}' left posture envelope at point "
                        f"{point_index} ({error:.3f}rad > {tolerance:.3f}rad)",
                    )

        return True, ""

    def _make_posture_lock_context(
        self,
        logger,
    ) -> tuple[dict[str, float] | None, Constraints | None]:
        """Create anchor positions and path constraints for the current posture."""
        if not self._posture_lock.enabled:
            return None, None

        anchor_positions = self._get_current_group_joint_positions()
        missing = [
            name
            for name in self._posture_lock.joint_names
            if name not in anchor_positions
        ]
        if missing:
            logger.error(
                "Posture lock could not anchor the current branch because "
                f"these joints are missing from the planning group: {missing}"
            )
            return None, None

        constraints = self._build_joint_constraints(
            anchor_positions,
            self._posture_lock.joint_names,
            self._posture_lock.tolerances_rad,
        )
        return anchor_positions, constraints

    def _trajectory_respects_current_posture(
        self,
        trajectory,
        anchor_positions: dict[str, float],
        logger,
    ) -> bool:
        """Validate the planned joint trajectory against the current posture lock."""
        joint_trajectory = self._extract_joint_trajectory(trajectory)
        ok, reason = self._trajectory_respects_posture_lock(
            joint_trajectory,
            anchor_positions,
            self._posture_lock.joint_names,
            self._posture_lock.tolerances_rad,
        )
        if not ok:
            logger.warn(f"Rejected trajectory outside posture lock: {reason}")
        return ok

    @property
    def is_ready(self) -> bool:
        return self.robot is not None

    def shutdown(self):
        """Release MoveItPy resources. Safe to call multiple times."""
        if self.robot is not None:
            try:
                self.robot.shutdown()
            except Exception:
                pass
            self.robot = None
            self.arm = None
            self.planning_scene_monitor = None

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
        posture_anchor: dict[str, float] | None = None,
        validate_posture_lock: bool = False,
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

        logger.debug(
            f"Planning {planner} vel={velocity_scaling:.3f} acc={acceleration_scaling:.3f}")

        plan_result = self.arm.plan(single_plan_parameters=plan_params)

        if plan_result and plan_result.trajectory:
            if validate_posture_lock and posture_anchor is not None:
                if not self._trajectory_respects_current_posture(
                    plan_result.trajectory, posture_anchor, logger
                ):
                    return False
            self.robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(sleep_time)
            return True

        logger.error("Planning failed!")
        if diagnostics:
            logger.error(
                f"  ee={diagnostics.get('ee_link', '-')} "
                f"planner={planner} vel={velocity_scaling:.3f} acc={acceleration_scaling:.3f}\n"
                f"  target={diagnostics.get('target_pose', '-')}\n"
                f"  resolved={diagnostics.get('resolved_pose', '-')}\n"
                f"  current={diagnostics.get('current_pose', '-')}")
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
        enforce_posture_lock: bool | None = None,
    ) -> bool:
        """Move end-effector to *target_pose* with optional TCP offset resolution."""
        resolved = resolve_tcp_fn(target_pose) if resolve_tcp_fn else target_pose

        vel = velocity_scaling if velocity_scaling is not None else default_velocity_scaling
        acc = acceleration_scaling if acceleration_scaling is not None else default_acceleration_scaling
        posture_enabled = (
            self._posture_lock.enabled
            if enforce_posture_lock is None
            else bool(enforce_posture_lock)
        )

        # Build diagnostics lazily — only used on failure
        def _diag():
            current_diag = "n/a"
            try:
                current_diag = self._format_raw_pose(
                    self.get_current_pose(end_effector_link))
            except Exception:
                pass
            return {
                "ee_link": end_effector_link,
                "target_pose": self._format_pose(target_pose),
                "resolved_pose": self._format_pose(resolved),
                "current_pose": current_diag,
            }

        ok = self._move_to_pose_once(
            resolved_pose=resolved,
            end_effector_link=end_effector_link,
            logger=logger,
            sleep_time=sleep_time,
            velocity_scaling=vel,
            acceleration_scaling=acc,
            planner_id=planner_id,
            posture_enabled=posture_enabled,
        )

        if not ok:
            logger.error(
                f"Planning step failed: {move_label or 'unnamed'}")
            diag = _diag()
            logger.error(
                f"  target={diag['target_pose']}\n"
                f"  resolved={diag['resolved_pose']}\n"
                f"  current={diag['current_pose']}")
        return ok

    def move_to_named_target(
        self,
        target_name: str,
        end_effector_link: str,
        logger,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        planner_id: str | None = None,
        default_velocity_scaling: float = 0.3,
        default_acceleration_scaling: float = 0.3,
    ) -> bool:
        """Move to a named target (group state from SRDF)."""
        if not self.robot or not self.arm:
            logger.error("MoveIt not initialized")
            return False

        vel = velocity_scaling if velocity_scaling is not None else default_velocity_scaling
        acc = acceleration_scaling if acceleration_scaling is not None else default_acceleration_scaling
        posture_enabled = self._posture_lock.enabled

        posture_anchor = None
        path_constraints = None
        if posture_enabled:
            posture_anchor, path_constraints = self._make_posture_lock_context(logger)
            if posture_anchor is None or path_constraints is None:
                return False

        self.arm.set_start_state_to_current_state()
        if path_constraints is not None:
            self.arm.set_path_constraints(path_constraints)

        try:
            logger.info(f"Setting goal state to named target: {target_name}")
            self.arm.set_goal_state(configuration_name=target_name)
            success = self.plan_and_execute(
                logger,
                sleep_time=0.0,
                velocity_scaling=vel,
                acceleration_scaling=acc,
                planner_id=planner_id,
                diagnostics=None,
                posture_anchor=posture_anchor,
                validate_posture_lock=posture_enabled,
            )
            if success:
                logger.info(f"Successfully moved to named target: {target_name}")
            else:
                logger.error(f"Failed to move to named target: {target_name}")
            return success
        except Exception as e:
            logger.error(f"Exception while moving to named target '{target_name}': {e}")
            return False
        finally:
            if path_constraints is not None:
                self.arm.set_path_constraints(Constraints())

    def _move_to_pose_once(
        self,
        *,
        resolved_pose: PoseStamped,
        end_effector_link: str,
        logger,
        sleep_time: float,
        velocity_scaling: float,
        acceleration_scaling: float,
        planner_id: str | None,
        posture_enabled: bool,
    ) -> bool:
        """Plan and execute a single pose goal attempt."""
        posture_anchor = None
        path_constraints = None
        if posture_enabled:
            posture_anchor, path_constraints = self._make_posture_lock_context(
                logger
            )
            if posture_anchor is None or path_constraints is None:
                return False

        self.arm.set_start_state_to_current_state()
        if path_constraints is not None:
            self.arm.set_path_constraints(path_constraints)

        try:
            self.arm.set_goal_state(
                pose_stamped_msg=resolved_pose,
                pose_link=end_effector_link,
            )
            return self.plan_and_execute(
                logger,
                sleep_time=sleep_time,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
                planner_id=planner_id,
                diagnostics=None,
                posture_anchor=posture_anchor,
                validate_posture_lock=posture_enabled,
            )
        finally:
            if path_constraints is not None:
                self.arm.set_path_constraints(Constraints())

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
        enforce_posture_lock: bool | None = None,
    ) -> bool:
        """Move end-effector relative to current position."""
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
            enforce_posture_lock=enforce_posture_lock,
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
