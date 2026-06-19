import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TwistStamped

from robin_interfaces.msg import WelderData, BeadGeometry

_FRAME = 'weld_torch_tip'
_NS = 'welding_bead'


class WeldingVisNode(Node):
    def __init__(self):
        super().__init__('welding_vis_node')

        self._height_mm = 0.0
        self._width_mm = 0.0
        self._area_mm2 = 0.0
        self._voltage_v = 0.0
        self._current_a = 0.0
        self._wire_feed_mpm = 0.0

        self._pub = self.create_publisher(MarkerArray, '/welding/visualization', 10)

        self.create_subscription(
            BeadGeometry, '/welding/bead_geometry', self._cb_geometry, 10)
        self.create_subscription(
            WelderData, '/welder/data', self._cb_welder, 10)
        self.create_subscription(
            TwistStamped, '/tool_velocity', self._cb_velocity, 10)

        self.create_timer(0.1, self._publish)
        self.get_logger().info('Welding visualization node started.')

    def _cb_geometry(self, msg):
        self._height_mm = msg.height_mm
        self._width_mm = msg.width_mm
        self._area_mm2 = msg.cross_sectional_area_mm2

    def _cb_welder(self, msg):
        self._voltage_v = msg.voltage_v
        self._current_a = msg.current_a
        self._wire_feed_mpm = msg.wire_feed_speed_mpm

    def _cb_velocity(self, msg):
        pass

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # Bead cylinder — scaled to cross-section, lifted half its height above tip
        cyl = Marker()
        cyl.header.stamp = now
        cyl.header.frame_id = _FRAME
        cyl.ns = _NS
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        w = max(self._width_mm / 1000.0, 0.001)
        h = max(self._height_mm / 1000.0, 0.001)
        cyl.scale.x = w
        cyl.scale.y = w
        cyl.scale.z = h
        cyl.pose.position.z = h / 2.0
        cyl.pose.orientation.w = 1.0
        cyl.color.r = 1.0
        cyl.color.g = 0.65
        cyl.color.b = 0.0
        cyl.color.a = 0.85
        arr.markers.append(cyl)

        # Bead geometry text
        txt_bead = Marker()
        txt_bead.header.stamp = now
        txt_bead.header.frame_id = _FRAME
        txt_bead.ns = _NS
        txt_bead.id = 1
        txt_bead.type = Marker.TEXT_VIEW_FACING
        txt_bead.action = Marker.ADD
        txt_bead.pose.position.z = 0.06
        txt_bead.pose.orientation.w = 1.0
        txt_bead.scale.z = 0.025
        txt_bead.color.r = 1.0
        txt_bead.color.g = 1.0
        txt_bead.color.b = 1.0
        txt_bead.color.a = 1.0
        txt_bead.text = (
            f'H:{self._height_mm:.2f}mm  W:{self._width_mm:.2f}mm\n'
            f'Area:{self._area_mm2:.2f}mm\u00b2'
        )
        arr.markers.append(txt_bead)

        # Welder params text
        txt_weld = Marker()
        txt_weld.header.stamp = now
        txt_weld.header.frame_id = _FRAME
        txt_weld.ns = _NS
        txt_weld.id = 2
        txt_weld.type = Marker.TEXT_VIEW_FACING
        txt_weld.action = Marker.ADD
        txt_weld.pose.position.z = 0.10
        txt_weld.pose.orientation.w = 1.0
        txt_weld.scale.z = 0.020
        txt_weld.color.r = 0.9
        txt_weld.color.g = 0.9
        txt_weld.color.b = 0.3
        txt_weld.color.a = 1.0
        txt_weld.text = (
            f'{self._voltage_v:.1f}V  {self._current_a:.0f}A  '
            f'WFS:{self._wire_feed_mpm:.1f}m/min'
        )
        arr.markers.append(txt_weld)

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WeldingVisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
