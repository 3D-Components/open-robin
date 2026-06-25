"""TCP mode switching and target-pose resolution for multi-TCP setups.

This is a data helper: it does NOT create any ROS2 interfaces itself.
All service clients, subscriptions, and TF objects must be created by
the parent node and passed into the constructor.
"""

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation

from robin_interfaces.srv import SetTcpMode, SetCtwd
from robin_core.utils import wait_for_future

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TcpHelper:
    """Manages TCP mode state and provides pose-offset resolution.

    Tracks the currently active TCP via subscription callbacks (registered
    by the parent node). Provides :meth:`resolve_target` to adjust goal
    poses when planning to a TCP other than *wire_tip*.

    All ROS2 interfaces (service clients, subscriptions, TF) must be
    created by the parent node and passed in.
    """

    def __init__(self, node, *, clients: dict, tf_buffer: tf2_ros.Buffer,
                 default_ctwd: float = 0.015):
        self._node = node
        self.active_frame: str = "wire_tip"
        self.current_ctwd: float = default_ctwd

        self._set_mode_client = clients['tcp_set_mode']
        self._set_ctwd_client = clients['tcp_set_ctwd']

        self._tf_buffer = tf_buffer

    @property
    def tf_buffer(self) -> tf2_ros.Buffer:
        """Expose TF buffer for external lookups (e.g. calibration)."""
        return self._tf_buffer

    # -- callbacks (registered by parent node) --------------------------------
    def frame_cb(self, msg: String):
        self.active_frame = msg.data

    def ctwd_cb(self, msg: Float32):
        self.current_ctwd = msg.data

    # -- public API ----------------------------------------------------------
    def set_mode(self, mode: str) -> bool:
        """Switch TCP mode via tcp/set_mode service.

        Returns True on success.
        """
        if not self._set_mode_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn("tcp/set_mode service not available")
            return False

        request = SetTcpMode.Request()
        request.mode = mode

        future = self._set_mode_client.call_async(request)
        result = wait_for_future(self._node, future, timeout_sec=5.0)

        if result is not None and result.success:
            self.active_frame = result.active_frame
            self._node.get_logger().info(
                f"TCP mode set to '{mode}' (frame: {self.active_frame})")
            return True

        msg = result.message if result else "timeout"
        self._node.get_logger().error(f"Failed to set TCP mode: {msg}")
        return False

    def set_ctwd(self, value_m: float, calibrated: bool = False) -> bool:
        """Set CTWD via tcp/set_ctwd service."""
        if not self._set_ctwd_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn("tcp/set_ctwd service not available")
            return False

        request = SetCtwd.Request()
        request.data = float(value_m)
        request.calibrated = calibrated

        future = self._set_ctwd_client.call_async(request)
        result = wait_for_future(self._node, future, timeout_sec=5.0)
        if result is not None and result.success:
            self.current_ctwd = float(value_m)
            self._node.get_logger().info(
                f"CTWD set to {value_m * 1000:.1f}mm")
            return True

        msg = result.message if result else "timeout"
        self._node.get_logger().error(f"Failed to set CTWD: {msg}")
        return False

    def resolve_target(self, target_pose: PoseStamped) -> PoseStamped:
        """Adjust *target_pose* so the active TCP frame reaches the goal.

        When the active TCP is *wire_tip* the pose is returned unchanged.
        Otherwise the static TF offset wire_tip → active_frame is applied.
        """
        if self.active_frame == "wire_tip":
            return target_pose

        try:
            transform = self._tf_buffer.lookup_transform(
                self.active_frame, "wire_tip",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().error(
                f"TF lookup wire_tip → {self.active_frame} failed: {e}. "
                f"Falling back to wire_tip planning.")
            return target_pose

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        q_off = transform.transform.rotation

        # Keep commanded orientation unchanged and only move origin from active TCP
        # frame to wire_tip. For scanning (active_frame=laser_frame), this preserves
        # weld orientation while shifting TCP origin to the laser origin as requested.
        q_target = target_pose.pose.orientation
        wire_rot_world = Rotation.from_quat(
            [q_target.x, q_target.y, q_target.z, q_target.w])
        active_to_wire_rot = Rotation.from_quat(
            [q_off.x, q_off.y, q_off.z, q_off.w])
        wire_to_active_rot = active_to_wire_rot.inv()
        active_rot_world = wire_rot_world * wire_to_active_rot
        offset_world = active_rot_world.apply([tx, ty, tz])

        adjusted = PoseStamped()
        adjusted.header = target_pose.header
        adjusted.pose.position.x = target_pose.pose.position.x + offset_world[0]
        adjusted.pose.position.y = target_pose.pose.position.y + offset_world[1]
        adjusted.pose.position.z = target_pose.pose.position.z + offset_world[2]
        adjusted.pose.orientation = target_pose.pose.orientation

        self._node.get_logger().debug(
            f"TCP offset for {self.active_frame}: "
            f"({tx:.4f}, {ty:.4f}, {tz:.4f}) → world "
            f"({offset_world[0]:.4f}, {offset_world[1]:.4f}, {offset_world[2]:.4f})")

        return adjusted
