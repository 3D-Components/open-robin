#!/usr/bin/env python3
"""
TCP Manager Node — Tool Center Point management for ROBIN.

Manages the active TCP (Tool Center Point) and publishes the wire_tip
frame on /tf_static so that CTWD calibration updates take effect
without restarting robot_state_publisher.

Responsibilities:
- Publishes contact_tip → wire_tip transform on /tf_static
  (re-published only when CTWD changes)
- Tracks current CTWD value (updated by calibration)
- Tracks active TCP mode (welding → wire_tip, scanning → laser_frame)
- Publishes active TCP frame name on tcp/active_frame (latched)
- Optionally persists calibrated CTWD to file for restart recovery

Services:
- tcp/set_mode       (SetTcpMode)    — Switch between welding/scanning TCP
- tcp/set_ctwd   (SetCtwd)   — Update CTWD value (from calibration
                                        or operator).

Topics published (TRANSIENT_LOCAL / latched):
- tcp/active_frame        (String)   — Currently active TCP frame name
- tcp/ctwd            (Float32)  — Current CTWD value in meters
- tcp/ctwd_calibrated (Bool)     — Whether current CTWD is calibrated
- /tf_static               (TFMessage) — Static wire_tip transform
"""

import json
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from robin_interfaces.srv import SetTcpMode, SetCtwd
from robin_core.utils.tcp_utils import LATCHED_QOS
from robin_core.utils.time_utils import format_time_policy, is_sim_time_enabled


# Valid TCP modes and their corresponding TF frames
TCP_MODES = {
    'welding': 'wire_tip',
    'scanning': 'laser_frame',
}


class TcpManager(Node):
    """Manages TCP frame and CTWD for the ROBIN welding system."""

    def __init__(self):
        super().__init__('tcp_manager')

        self.declare_parameter('default_ctwd', 0.015)
        self.declare_parameter('default_mode', 'welding')
        self.declare_parameter('ctwd_file', '')

        self._ctwd = self.get_parameter('default_ctwd').value
        default_mode = self.get_parameter('default_mode').value
        self._ctwd_file = self.get_parameter('ctwd_file').value or ''

        if default_mode not in TCP_MODES:
            self.get_logger().warn(
                f"Invalid default_mode '{default_mode}', falling back to 'welding'")
            default_mode = 'welding'

        self._mode = default_mode
        self._active_frame = TCP_MODES[default_mode]
        self._ctwd_calibrated = False

        self._load_ctwd()

        self._cb_group = ReentrantCallbackGroup()

        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._active_frame_pub = self.create_publisher(
            String, 'tcp/active_frame', LATCHED_QOS)
        self._ctwd_pub = self.create_publisher(
            Float32, 'tcp/ctwd', LATCHED_QOS)
        self._ctwd_calibrated_pub = self.create_publisher(
            Bool, 'tcp/ctwd_calibrated', LATCHED_QOS)

        self._set_mode_srv = self.create_service(
            SetTcpMode, 'tcp/set_mode',
            self._set_mode_callback,
            callback_group=self._cb_group)
        self._set_ctwd_srv = self.create_service(
            SetCtwd, 'tcp/set_ctwd',
            self._set_ctwd_callback,
            callback_group=self._cb_group)

        self.get_logger().info(
            f'Configured: mode={self._mode}, ctwd={self._ctwd * 1000:.1f}mm')
        self.get_logger().info(
            format_time_policy(
                self,
                ros_time_inputs=['/clock', '/tf_static'],
                steady_time_inputs=['service calls', 'CTWD persistence'],
            )
        )

        if is_sim_time_enabled(self) and self.get_clock().now().nanoseconds <= 0:
            self.get_logger().warn(
                'Publishing /tf_static before sim time has started; '
                'downstream TF consumers may not resolve until `/clock` advances'
            )
        self._publish_wire_tip_tf()
        self._publish_topics()

        self.get_logger().info(
            f'Activated: mode={self._mode}, '
            f'active_frame={self._active_frame}, '
            f'ctwd={self._ctwd * 1000:.1f}mm')

    # ------------------------------------------------------------------
    # Static TF publish — only called when CTWD changes
    # ------------------------------------------------------------------
    def _publish_wire_tip_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'contact_tip'
        t.child_frame_id = 'wire_tip'
        t.transform.translation.z = self._ctwd
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Topic publish — only called on state change
    # ------------------------------------------------------------------
    def _publish_topics(self):
        frame_msg = String()
        frame_msg.data = self._active_frame
        self._active_frame_pub.publish(frame_msg)

        ctwd_msg = Float32()
        ctwd_msg.data = float(self._ctwd)
        self._ctwd_pub.publish(ctwd_msg)

        cal_msg = Bool()
        cal_msg.data = self._ctwd_calibrated
        self._ctwd_calibrated_pub.publish(cal_msg)

    # ------------------------------------------------------------------
    # CTWD persistence
    # ------------------------------------------------------------------
    def _load_ctwd(self):
        if not self._ctwd_file:
            return
        try:
            with open(self._ctwd_file, 'r') as f:
                data = json.load(f)
            self._ctwd = float(data['ctwd'])
            self._ctwd_calibrated = bool(data.get('calibrated', False))
            self.get_logger().info(
                f'Loaded persisted CTWD: {self._ctwd * 1000:.1f}mm '
                f'(calibrated={self._ctwd_calibrated})')
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Failed to load CTWD file: {e}')

    def _save_ctwd(self):
        if not self._ctwd_file:
            return
        try:
            os.makedirs(os.path.dirname(self._ctwd_file) or '.', exist_ok=True)
            with open(self._ctwd_file, 'w') as f:
                json.dump({'ctwd': self._ctwd,
                           'calibrated': self._ctwd_calibrated}, f)
        except Exception as e:
            self.get_logger().warn(f'Failed to save CTWD file: {e}')

    # ------------------------------------------------------------------
    # Service: /tcp/set_mode
    # ------------------------------------------------------------------
    def _set_mode_callback(self, request: SetTcpMode.Request,
                           response: SetTcpMode.Response) -> SetTcpMode.Response:
        mode = request.mode.lower().strip()

        if mode not in TCP_MODES:
            response.success = False
            response.active_frame = self._active_frame
            response.message = (
                f"Unknown mode '{mode}'. "
                f"Valid modes: {', '.join(TCP_MODES.keys())}")
            self.get_logger().warn(response.message)
            return response

        old_mode = self._mode
        self._mode = mode
        self._active_frame = TCP_MODES[mode]

        response.success = True
        response.active_frame = self._active_frame
        response.message = (
            f"TCP mode changed: {old_mode} → {mode} "
            f"(frame: {self._active_frame})")
        self.get_logger().info(response.message)

        # Invalidate CTWD calibration on mode change
        if old_mode != mode:
            self._ctwd_calibrated = False
            self.get_logger().info(
                'CTWD calibration invalidated (TCP mode changed)')

        self._publish_topics()
        return response

    # ------------------------------------------------------------------
    # Service: /tcp/set_ctwd
    # ------------------------------------------------------------------
    def _set_ctwd_callback(self, request: SetCtwd.Request,
                               response: SetCtwd.Response) -> SetCtwd.Response:
        new_ctwd = request.data

        if new_ctwd < 0.0:
            response.success = False
            response.message = f"Invalid CTWD {new_ctwd:.4f}m (must be >= 0)"
            self.get_logger().warn(response.message)
            return response

        if new_ctwd > 0.050:  # >50mm — probably an error
            self.get_logger().warn(
                f"Large CTWD value {new_ctwd * 1000:.1f}mm — verify this is correct")

        old_ctwd = self._ctwd
        self._ctwd = new_ctwd
        self._ctwd_calibrated = request.calibrated

        response.success = True
        response.message = (
            f"CTWD updated: {old_ctwd * 1000:.1f}mm → "
            f"{new_ctwd * 1000:.1f}mm"
            f" (calibrated={request.calibrated})")
        self.get_logger().info(response.message)

        self._publish_wire_tip_tf()
        self._publish_topics()
        self._save_ctwd()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TcpManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
