import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from robin_interfaces.msg import BeadGeometry
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor
from collections import deque


class WeldProfileProcessor(Node):
    def __init__(self):
        super().__init__('weld_profile_processor')
        
        # Declare parameters
        self.declare_parameter('filter_window', 5)
        self.declare_parameter('input_topic', '/robin/pointcloud')
        self.declare_parameter('output_topic', '/robin/weld_dimensions')
        
        # Get parameters
        filter_window = self.get_parameter('filter_window').get_parameter_value().integer_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # Moving average filter
        self.filter_window = max(1, filter_window)
        self._width_history = deque(maxlen=self.filter_window)
        self._height_history = deque(maxlen=self.filter_window)
        self._area_history = deque(maxlen=self.filter_window)
        
        # Subscriber to pointcloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )
        
        # Publisher for weld dimensions
        self.publisher = self.create_publisher(
            BeadGeometry,
            output_topic,
            10
        )
        
        self.get_logger().info(
            f'Weld Profile Processor Node initialized '
            f'(filter_window={self.filter_window}, '
            f'input={input_topic}, output={output_topic})'
        )
    
    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points.size == 0:
                self.get_logger().warn('Received empty pointcloud')
                return
            
            # Compute weld dimensions
            width, height, area = self.compute_weld_dimensions(points)
            
            # Publish results
            msg_out = BeadGeometry()
            msg_out.header = msg.header
            msg_out.width_mm = float(width)
            msg_out.height_mm = float(height)
            msg_out.cross_sectional_area_mm2 = float(area)
            self.publisher.publish(msg_out)
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # list() consumes the generator in C, which is much faster than a Python for-loop
        return np.array(list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))))
    
    def compute_weld_dimensions(self, points):
        """
        Compute weld width, height, and area using SOTA 2D profilometry techniques:
        1. Y-axis sorting & Signal Smoothing
        2. Robust Base Plate Fitting (RANSAC)
        3. Precise Toe Detection (Left/Right boundaries)
        """
        if len(points) < 20:
            return 0.0, 0.0, 0.0
        
        try:
            # Convert to mm
            points = points * 1000.0
            
            # 1. Sort by Y to ensure sequential 1D processing
            sort_idx = np.argsort(points[:, 1])
            y = points[sort_idx, 1]
            z = points[sort_idx, 2]
            
            # 2. Smooth the Z profile to remove laser speckle/spikes (Moving Average)
            window_size = 5
            kernel = np.ones(window_size) / window_size
            z_smooth = np.convolve(z, kernel, mode='same')
            # Restore edges lost in convolution
            z_smooth[:2] = z[:2]
            z_smooth[-2:] = z[-2:]

            # 3. Fit base plane using RANSAC on the smoothed data
            y_reshape = y.reshape(-1, 1)
            ransac = RANSACRegressor(min_samples=20, residual_threshold=0.2)
            ransac.fit(y_reshape, z_smooth)
            
            # 4. Calculate height relative to the base plate
            z_base = ransac.predict(y_reshape)
            z_relative = z_smooth - z_base
            
            # 5. Toe Detection: Find where the profile leaves the base plate
            # We define the bead as regions deviating > 0.3mm from the flat plate
            bead_threshold = 0.3
            is_bead = z_relative > bead_threshold
            
            # Find contiguous segments (edges where boolean array changes)
            edges = np.diff(is_bead.astype(int))
            starts = np.where(edges == 1)[0] + 1
            ends = np.where(edges == -1)[0] + 1
            
            # Handle edge cases (bead touches the very edge of the scanner FOV)
            if is_bead[0]:
                starts = np.insert(starts, 0, 0)
            if is_bead[-1]:
                ends = np.append(ends, len(is_bead))
                
            if len(starts) == 0:
                return 0.0, 0.0, 0.0
                
            # 6. Select the main bead (the widest contiguous segment)
            segment_lengths = ends - starts
            main_bead_idx = np.argmax(segment_lengths)
            start_idx = starts[main_bead_idx]
            end_idx = ends[main_bead_idx] - 1  # Inclusive end
            
            # Reject false positives (e.g., noise segments narrower than 2mm)
            if (y[end_idx] - y[start_idx]) < 2.0:
                return 0.0, 0.0, 0.0
                
            # 7. Extract precise dimensions between the detected toes
            y_bead = y[start_idx:end_idx+1]
            z_rel_bead = z_relative[start_idx:end_idx+1]
            
            width_raw = y_bead[-1] - y_bead[0]
            height_raw = np.max(z_rel_bead)
            area_raw = np.trapz(z_rel_bead, y_bead)
            
            # 8. Apply moving average filter over time
            self._width_history.append(width_raw)
            self._height_history.append(height_raw)
            self._area_history.append(area_raw)
            
            width = np.mean(self._width_history)
            height = np.mean(self._height_history)
            area = np.mean(self._area_history)
            
            return float(width), float(height), float(area)
            
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
