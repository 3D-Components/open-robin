import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FroniusWeldingDataNode(Node):
    def __init__(self):
        super().__init__('fronius_welding_data')
        self.publisher_ = self.create_publisher(String, 'rt/welding/measurement', 10)
        period = 0.5
        self.timer = self.create_timer(period, self.publish_mock)
        self.get_logger().info('Fronius welding data node started, publishing to rt/welding/measurement')

    def publish_mock(self):
        # Simulate a small stream of welding measurements
        payload = {
            'process_id': 'sim-fronius',
            'timestamp': self.get_clock().now().to_msg().sec,
            'current': round(random.uniform(90.0, 130.0), 2),
            'voltage': round(random.uniform(18.0, 24.0), 2),
            'wire_speed': round(random.uniform(7.0, 12.0), 2),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FroniusWeldingDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

