import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from robin_interfaces.msg import BeadGeometry, ActiveBead, WeldProgression
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn

from robin_core.sensor.profile_analysis import analyse_weld_profile


class WeldProfileProcessor(Node):
    def __init__(self):
        super().__init__('weld_profile_processor')
        self._active_bead_id = ""
        self._current_progression = 0.0
        self._is_welding = False
        self._subscriptions = []

        self._subscriptions = [
            self.create_subscription(
                PointCloud2, "robin/pointcloud",
                self.pointcloud_callback, qos_profile_sensor_data),
            self.create_subscription(
                ActiveBead, 'robin/data/active_bead',
                self._active_bead_callback, 10),
            self.create_subscription(
                WeldProgression, 'robin/data/progression',
                self._progression_callback, 10),
            self.create_subscription(
                Bool, 'robin/data/is_welding',
                self._welding_state_callback, 10),
        ]
        self.publisher = self.create_publisher(BeadGeometry, "robin/weld_dimensions", 10)
        self.get_logger().info('Configured: _subscriptions + publisher ready')
    
    def _active_bead_callback(self, msg: ActiveBead):
        """Track active bead ID for stamping geometry output."""
        self._active_bead_id = msg.bead_id
    
    def _progression_callback(self, msg: WeldProgression):
        """Track latest progression value for stamping geometry output."""
        self._current_progression = msg.progression
    
    def _welding_state_callback(self, msg: Bool):
        """Track welding state — suppress pointcloud processing while arc is on."""
        self._is_welding = msg.data
    
    def pointcloud_callback(self, msg):
        # Don't process sensor data while welding — sensor should be off
        if self._is_welding:
            return

        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points.size == 0:
                self.get_logger().debug('Received empty pointcloud')
                return

            self.get_logger().debug(f'Pointcloud: {points.shape[0]} points')
            
            # Compute weld dimensions
            width, height, toe_angle = self.compute_weld_dimensions(points)
            
            # Publish results
            msg_out = BeadGeometry()
            msg_out.header = msg.header
            msg_out.bead_id = self._active_bead_id
            msg_out.progression = self._current_progression
            msg_out.width_mm = float(width)
            msg_out.height_mm = float(height)
            msg_out.toe_angle_rad = float(toe_angle)
            self.publisher.publish(msg_out)
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy (N, 3) float array."""
        structured = np.array(
            list(pc2.read_points(cloud_msg, skip_nans=True,
                                field_names=("x", "y", "z"))))
        if structured.size == 0:
            return structured
        return rfn.structured_to_unstructured(structured).astype(np.float64)
    
    def compute_weld_dimensions(self, points):
        """Compute weld width, height and toe angle using RANSAC profilometry."""
        if len(points) < 20:
            return 0.0, 0.0, 0.0
        try:
            points_mm = points * 1000.0
            sort_idx = np.argsort(points_mm[:, 1])
            result = analyse_weld_profile(points_mm[sort_idx])
            if result['width'] is None:
                return 0.0, 0.0, 0.0
            toe_angle = result['toe_angle_rad'] if result['toe_angle_rad'] is not None else 0.0
            return float(result['width']), float(result['height']), float(toe_angle)
        except Exception as e:
            self.get_logger().error(f'Error computing weld dimensions: {e}')
            return 0.0, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    
    weld_processor = WeldProfileProcessor()
    
    try:
        rclpy.spin(weld_processor)
    except KeyboardInterrupt:
        pass
    finally:
        weld_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
