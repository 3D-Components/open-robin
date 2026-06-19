#!/usr/bin/env python3
"""
TCP Manager Node — Dynamic Tool Center Point management for ROBIN.

Manages the active TCP (Tool Center Point) and publishes the wire_tip
frame dynamically on /tf so that stickout calibration updates take
effect immediately without restarting robot_state_publisher.

Responsibilities:
- Publishes contact_tip → wire_tip transform on /tf at configured rate
  (overrides the static URDF version from robot_state_publisher)
- Tracks current stickout value (updated by calibration)
- Tracks active TCP mode (welding → wire_tip, scanning → laser_frame, contact_tip → contact_tip)
- Publishes active TCP frame name on /tcp/active_frame

Services:
- /tcp/set_mode       (SetTcpMode)   — Switch between welding/scanning TCP
- /tcp/set_stickout   (SetFloat32)   — Update stickout value (from calibration)

Topics published:
- /tcp/active_frame   (String)       — Currently active TCP frame name
- /tcp/stickout       (Float32)      — Current stickout value in meters
- /tf                 (TFMessage)    — Dynamic wire_tip transform
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger

from robin_interfaces.srv import SetTcpMode, SetFloat32 as SetFloat32Srv


# Valid TCP modes and their corresponding TF frames
TCP_MODES = {
    'welding': 'wire_tip',
    'scanning': 'laser_frame',
    'contact_tip': 'contact_tip',
}


class TcpManager(Node):
    """Manages dynamic TCP frame and stickout for the ROBIN welding system."""

    def __init__(self):
        super().__init__('tcp_manager')

        # Parameters
        self.declare_parameter('default_stickout', 0.015)  # 15 mm
        self.declare_parameter('publish_rate', 100.0)       # Hz
        self.declare_parameter('default_mode', 'welding')

        self._stickout = self.get_parameter('default_stickout').value
        publish_rate = self.get_parameter('publish_rate').value
        default_mode = self.get_parameter('default_mode').value

        if default_mode not in TCP_MODES:
            self.get_logger().warn(
                f"Invalid default_mode '{default_mode}', falling back to 'welding'")
            default_mode = 'welding'

        self._mode = default_mode
        self._active_frame = TCP_MODES[default_mode]
        self._stickout_calibrated = False  # Set True after successful calibration

        self._cb_group = ReentrantCallbackGroup()

        # TF broadcaster for dynamic wire_tip frame
        self._tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self._active_frame_pub = self.create_publisher(
            String, '/tcp/active_frame', 10)
        self._stickout_pub = self.create_publisher(
            Float32, '/tcp/stickout', 10)
        self._stickout_calibrated_pub = self.create_publisher(
            Bool, '/tcp/stickout_calibrated', 10)

        # Services
        self._set_mode_srv = self.create_service(
            SetTcpMode, '/tcp/set_mode',
            self._set_mode_callback,
            callback_group=self._cb_group)
        self._set_stickout_srv = self.create_service(
            SetFloat32Srv, '/tcp/set_stickout',
            self._set_stickout_callback,
            callback_group=self._cb_group)
        self._mark_stickout_calibrated_srv = self.create_service(
            Trigger, '/tcp/mark_stickout_calibrated',
            self._mark_stickout_calibrated_callback,
            callback_group=self._cb_group)

        # Timer for publishing TF and topics
        period = 1.0 / publish_rate
        self._timer = self.create_timer(period, self._publish_tick)

        self.get_logger().info(
            f'TCP Manager started: mode={self._mode}, '
            f'active_frame={self._active_frame}, '
            f'stickout={self._stickout * 1000:.1f} mm')
        self.get_logger().info(
            'Services: /tcp/set_mode, /tcp/set_stickout, /tcp/mark_stickout_calibrated')

    # ------------------------------------------------------------------
    # Timer callback — publishes TF + topics each tick
    # ------------------------------------------------------------------
    def _publish_tick(self):
        now = self.get_clock().now().to_msg()

        # Publish dynamic wire_tip transform (overrides static URDF version)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'contact_tip'
        t.child_frame_id = 'wire_tip'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self._stickout
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

        # Publish active frame name
        frame_msg = String()
        frame_msg.data = self._active_frame
        self._active_frame_pub.publish(frame_msg)

        # Publish current stickout
        stickout_msg = Float32()
        stickout_msg.data = float(self._stickout)
        self._stickout_pub.publish(stickout_msg)

        # Publish stickout calibration status
        cal_msg = Bool()
        cal_msg.data = self._stickout_calibrated
        self._stickout_calibrated_pub.publish(cal_msg)

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

        # Invalidate stickout calibration on mode change
        if old_mode != mode:
            self._stickout_calibrated = False
            self.get_logger().info(
                'Stickout calibration invalidated (TCP mode changed)')

        return response

    # ------------------------------------------------------------------
    # Service: /tcp/set_stickout
    # ------------------------------------------------------------------
    def _set_stickout_callback(self, request: SetFloat32Srv.Request,
                               response: SetFloat32Srv.Response) -> SetFloat32Srv.Response:
        new_stickout = request.data

        if new_stickout < 0.0:
            response.success = False
            response.message = f"Invalid stickout {new_stickout:.4f}m (must be >= 0)"
            self.get_logger().warn(response.message)
            return response

        if new_stickout > 0.050:  # >50mm — probably an error
            self.get_logger().warn(
                f"Large stickout value {new_stickout * 1000:.1f}mm — verify this is correct")

        old_stickout = self._stickout
        self._stickout = new_stickout

        response.success = True
        response.message = (
            f"Stickout updated: {old_stickout * 1000:.1f}mm → "
            f"{new_stickout * 1000:.1f}mm")
        self.get_logger().info(response.message)

        # Manual/setpoint changes invalidate calibration
        self._stickout_calibrated = False
        self.get_logger().info('Stickout calibration invalidated (value changed)')

        return response

    def _mark_stickout_calibrated_callback(self, request: Trigger.Request,
                                           response: Trigger.Response) -> Trigger.Response:
        self._stickout_calibrated = True
        response.success = True
        response.message = 'Stickout marked as calibrated'
        self.get_logger().info(response.message)
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
