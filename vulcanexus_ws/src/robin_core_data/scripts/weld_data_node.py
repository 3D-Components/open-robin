#!/usr/bin/env python3
"""
ROBIN Weld Data Node

Adds progression tracking to weld data from multiple sources.
Progression (0.0-1.0) indicates normalized position along the active bead,
calculated from TCP position relative to bead start/end points.

This allows downstream alignment of data from different sources
by matching progression values rather than timestamps.

Subscribes to:
- /fronius/display_* - Raw Fronius welder data
- /robin/data/active_bead - Active bead info (start/end points)
- TF (base_link -> weld_torch_tip)

Publishes:
- /robin/data/progression - Current progression along active bead
- /robin/data/fronius - Fronius data with progression added
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point
from robin_interfaces.msg import WeldProgression, FroniusSample, ActiveBead


class WeldDataNode(Node):
    """Adds progression tracking to weld data from multiple sources."""

    def __init__(self):
        super().__init__('weld_data_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('current_topic', '/fronius/display_current')
        self.declare_parameter('voltage_topic', '/fronius/display_voltage')
        self.declare_parameter('wire_feed_topic', '/fronius/display_wfs')
        self.declare_parameter('power_topic', '/fronius/display_power')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'weld_torch_tip')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.current_topic = self.get_parameter('current_topic').value
        self.voltage_topic = self.get_parameter('voltage_topic').value
        self.wire_feed_topic = self.get_parameter('wire_feed_topic').value
        self.power_topic = self.get_parameter('power_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Active bead state
        self._active_bead_id = ""
        self._bead_start = Point()
        self._bead_end = Point()
        self._bead_length = 0.0
        self._is_welding = False
        
        # Target parameters for active bead
        self._target_speed = 0.0
        self._target_current = 0.0
        self._target_voltage = 0.0
        self._target_wire_speed = 0.0
        
        # Latest Fronius data
        self._current = 0.0
        self._voltage = 0.0
        self._wire_feed_speed = 0.0
        self._power = 0.0
        
        # TF buffer for TCP position lookup
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        # Subscribers for Fronius data
        self._current_sub = self.create_subscription(
            Float32, self.current_topic, self._current_callback,
            10, callback_group=self.callback_group)
        self._voltage_sub = self.create_subscription(
            Float32, self.voltage_topic, self._voltage_callback,
            10, callback_group=self.callback_group)
        self._wire_feed_sub = self.create_subscription(
            Float32, self.wire_feed_topic, self._wire_feed_callback,
            10, callback_group=self.callback_group)
        self._power_sub = self.create_subscription(
            Float32, self.power_topic, self._power_callback,
            10, callback_group=self.callback_group)
        
        # Subscriber for active bead info
        self._active_bead_sub = self.create_subscription(
            ActiveBead, '/robin/data/active_bead', self._active_bead_callback,
            10, callback_group=self.callback_group)
        
        # Subscriber for welding state
        self._welding_state_sub = self.create_subscription(
            Bool, '/robin/data/is_welding', self._welding_state_callback,
            10, callback_group=self.callback_group)
        
        # Publishers
        self._progression_pub = self.create_publisher(
            WeldProgression, '/robin/data/progression', 10)
        self._fronius_pub = self.create_publisher(
            FroniusSample, '/robin/data/fronius', 10)
        
        # Timer for periodic publishing
        period = 1.0 / self.publish_rate
        self._publish_timer = self.create_timer(
            period, self._publish_data, callback_group=self.callback_group)
        
        self.get_logger().info(
            f'Weld data node started, publishing at {self.publish_rate}Hz')
        self.get_logger().info('  Progression: /robin/data/progression')
        self.get_logger().info('  Fronius: /robin/data/fronius')
    
    # -------------------------------------------------------------------------
    # Fronius data callbacks
    # -------------------------------------------------------------------------
    
    def _current_callback(self, msg: Float32):
        self._current = msg.data
    
    def _voltage_callback(self, msg: Float32):
        self._voltage = msg.data
    
    def _wire_feed_callback(self, msg: Float32):
        self._wire_feed_speed = msg.data
    
    def _power_callback(self, msg: Float32):
        self._power = msg.data
    
    # -------------------------------------------------------------------------
    # Bead tracking callbacks
    # -------------------------------------------------------------------------
    
    def _active_bead_callback(self, msg: ActiveBead):
        """Handle active bead info - called when a bead starts."""
        if msg.bead_id != self._active_bead_id:
            self.get_logger().info(
                f'Active bead: {msg.bead_id} (length={msg.length*1000:.1f}mm)')
        
        self._active_bead_id = msg.bead_id
        self._bead_start = msg.start_point
        self._bead_end = msg.end_point
        self._bead_length = msg.length
        self._target_speed = msg.target_speed
        self._target_current = msg.target_current
        self._target_voltage = msg.target_voltage
        self._target_wire_speed = msg.target_wire_speed
    
    def _welding_state_callback(self, msg: Bool):
        """Handle welding state changes."""
        if msg.data != self._is_welding:
            state_str = "ARC ON" if msg.data else "ARC OFF"
            self.get_logger().info(f'Welding state: {state_str}')
            
            # Clear active bead when welding stops
            if not msg.data:
                self._active_bead_id = ""
                self._bead_length = 0.0
        
        self._is_welding = msg.data
    
    # -------------------------------------------------------------------------
    # Progression calculation
    # -------------------------------------------------------------------------
    
    def _get_tcp_position(self) -> Point | None:
        """Get current TCP position from TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, rclpy.time.Time())
            
            pos = Point()
            pos.x = transform.transform.translation.x
            pos.y = transform.transform.translation.y
            pos.z = transform.transform.translation.z
            return pos
        except TransformException as ex:
            self.get_logger().debug(f'Could not get TCP transform: {ex}')
            return None
    
    def _calculate_progression(self, tcp_pos: Point) -> float:
        """Calculate progression (0.0-1.0) along the active bead.
        
        Uses projection of TCP position onto the bead line segment.
        """
        if self._bead_length <= 0:
            return 0.0
        
        # Vector from bead start to end
        bead_vec_x = self._bead_end.x - self._bead_start.x
        bead_vec_y = self._bead_end.y - self._bead_start.y
        bead_vec_z = self._bead_end.z - self._bead_start.z
        
        # Vector from bead start to TCP
        tcp_vec_x = tcp_pos.x - self._bead_start.x
        tcp_vec_y = tcp_pos.y - self._bead_start.y
        tcp_vec_z = tcp_pos.z - self._bead_start.z
        
        # Project TCP onto bead line: dot(tcp_vec, bead_vec) / |bead_vec|^2
        dot_product = (tcp_vec_x * bead_vec_x + 
                       tcp_vec_y * bead_vec_y + 
                       tcp_vec_z * bead_vec_z)
        bead_length_sq = self._bead_length * self._bead_length
        
        progression = dot_product / bead_length_sq
        
        # Clamp to [0.0, 1.0]
        return max(0.0, min(1.0, progression))
    
    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------
    
    def _publish_data(self):
        """Publish progression and Fronius data with progression."""
        now = self.get_clock().now()
        
        # Get TCP position
        tcp_pos = self._get_tcp_position()
        if tcp_pos is None:
            tcp_pos = Point()  # Default to origin if TF unavailable
        
        # Calculate progression
        progression = 0.0
        if self._is_welding and self._bead_length > 0:
            progression = self._calculate_progression(tcp_pos)
        
        # Publish progression message
        prog_msg = WeldProgression()
        prog_msg.header.stamp = now.to_msg()
        prog_msg.header.frame_id = self.base_frame
        prog_msg.bead_id = self._active_bead_id
        prog_msg.progression = progression
        prog_msg.is_welding = self._is_welding
        prog_msg.tcp_position = tcp_pos
        self._progression_pub.publish(prog_msg)
        
        # Publish Fronius data with progression
        fronius_msg = FroniusSample()
        fronius_msg.header.stamp = now.to_msg()
        fronius_msg.header.frame_id = self.base_frame
        fronius_msg.bead_id = self._active_bead_id
        fronius_msg.progression = progression
        fronius_msg.current = self._current
        fronius_msg.voltage = self._voltage
        fronius_msg.wire_feed_speed = self._wire_feed_speed
        fronius_msg.power = self._power
        self._fronius_pub.publish(fronius_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WeldDataNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
