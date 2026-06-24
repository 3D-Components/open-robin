import math
import sys
import types
from dataclasses import dataclass, field


def _install_motion_import_stubs():
    """Provide minimal ROS/MoveIt message shims when tests run unsourced."""
    if "geometry_msgs.msg" not in sys.modules:
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

        @dataclass
        class Quaternion:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0
            w: float = 1.0

        @dataclass
        class _Point:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0

        @dataclass
        class _Header:
            frame_id: str = ""
            stamp: object = None

        @dataclass
        class _Pose:
            position: _Point = field(default_factory=_Point)
            orientation: Quaternion = field(default_factory=Quaternion)

        @dataclass
        class PoseStamped:
            header: _Header = field(default_factory=_Header)
            pose: _Pose = field(default_factory=_Pose)

        geometry_msgs_msg.Quaternion = Quaternion
        geometry_msgs_msg.PoseStamped = PoseStamped
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules["geometry_msgs"] = geometry_msgs
        sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    if "moveit_msgs.msg" not in sys.modules:
        moveit_msgs = types.ModuleType("moveit_msgs")
        moveit_msgs_msg = types.ModuleType("moveit_msgs.msg")

        @dataclass
        class JointConstraint:
            joint_name: str = ""
            position: float = 0.0
            tolerance_above: float = 0.0
            tolerance_below: float = 0.0
            weight: float = 0.0

        @dataclass
        class Constraints:
            name: str = ""
            joint_constraints: list = field(default_factory=list)

        moveit_msgs_msg.JointConstraint = JointConstraint
        moveit_msgs_msg.Constraints = Constraints
        moveit_msgs.msg = moveit_msgs_msg
        sys.modules["moveit_msgs"] = moveit_msgs
        sys.modules["moveit_msgs.msg"] = moveit_msgs_msg

    if "moveit.planning" not in sys.modules:
        moveit = types.ModuleType("moveit")
        moveit_planning = types.ModuleType("moveit.planning")

        class DummyMoveItPy:
            class PlanningComponent:
                pass

            class PlanningSceneMonitor:
                pass

        class DummyPlanRequestParameters:
            def __init__(self, *_args, **_kwargs):
                self.planning_pipeline = ""
                self.planner_id = ""
                self.max_velocity_scaling_factor = 0.0
                self.max_acceleration_scaling_factor = 0.0

        moveit_planning.MoveItPy = DummyMoveItPy
        moveit_planning.PlanRequestParameters = DummyPlanRequestParameters
        moveit.planning = moveit_planning
        sys.modules["moveit"] = moveit
        sys.modules["moveit.planning"] = moveit_planning


_install_motion_import_stubs()

from robin_core.motion import MotionPlanner


@dataclass
class DummyJointTrajectoryPoint:
    positions: list[float] = field(default_factory=list)


@dataclass
class DummyJointTrajectory:
    joint_names: list[str] = field(default_factory=list)
    points: list[DummyJointTrajectoryPoint] = field(default_factory=list)


def _make_trajectory(joint_names, points):
    traj = DummyJointTrajectory()
    traj.joint_names = list(joint_names)
    traj.points = []
    for positions in points:
        traj.points.append(DummyJointTrajectoryPoint(list(positions)))
    return traj


def test_build_joint_constraints_uses_anchor_positions():
    anchor = {
        "shoulder_lift_joint": -1.2,
        "elbow_joint": 1.4,
        "wrist_1_joint": -1.0,
    }
    constraints = MotionPlanner._build_joint_constraints(
        anchor,
        ("shoulder_lift_joint", "elbow_joint", "wrist_1_joint"),
        (0.75, 0.50, 0.25),
    )

    assert constraints.name == "posture_lock"
    assert [c.joint_name for c in constraints.joint_constraints] == [
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
    ]
    assert constraints.joint_constraints[0].position == anchor["shoulder_lift_joint"]
    assert constraints.joint_constraints[1].tolerance_above == 0.50
    assert constraints.joint_constraints[2].tolerance_below == 0.25


def test_trajectory_respects_posture_lock_with_wrapped_angles():
    anchor = {
        "shoulder_lift_joint": math.pi - 0.05,
        "elbow_joint": 1.4,
    }
    traj = _make_trajectory(
        ["shoulder_lift_joint", "elbow_joint"],
        [
            [-math.pi + 0.04, 1.45],
            [math.pi - 0.02, 1.50],
        ],
    )

    ok, reason = MotionPlanner._trajectory_respects_posture_lock(
        traj,
        anchor,
        ("shoulder_lift_joint", "elbow_joint"),
        (0.20, 0.20),
    )

    assert ok is True
    assert reason == ""


def test_trajectory_rejects_branch_flip():
    anchor = {
        "shoulder_lift_joint": -1.2,
        "elbow_joint": 1.4,
    }
    traj = _make_trajectory(
        ["shoulder_lift_joint", "elbow_joint"],
        [
            [-1.1, 1.45],
            [0.2, -0.6],
        ],
    )

    ok, reason = MotionPlanner._trajectory_respects_posture_lock(
        traj,
        anchor,
        ("shoulder_lift_joint", "elbow_joint"),
        (0.50, 0.50),
    )

    assert ok is False
    assert "left posture envelope" in reason
