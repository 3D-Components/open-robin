import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import DBSCAN
from collections import deque


class WeldProfileProcessor(Node):
    def __init__(self):
        super().__init__('weld_profile_processor')
        
        # Declare parameters
        self.declare_parameter('filter_window', 5)
        
        # Get parameters
        filter_window = self.get_parameter('filter_window').get_parameter_value().integer_value
        
        # Moving average filter
        self.filter_window = max(1, filter_window)
        self._width_history = deque(maxlen=self.filter_window)
        self._height_history = deque(maxlen=self.filter_window)
        
        # Subscriber to pointcloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/robin/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for weld dimensions [width, height]
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/robin/weld_dimensions',
            10
        )
        
        self.get_logger().info(f'Weld Profile Processor Node initialized (filter_window={self.filter_window})')
    
    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points.size == 0:
                self.get_logger().warn('Received empty pointcloud')
                return
            
            # Compute weld dimensions
            width, height = self.compute_weld_dimensions(points)
            
            # Publish results
            dimensions_msg = Float32MultiArray()
            dimensions_msg.data = [width, height]
            self.publisher.publish(dimensions_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        points_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])
        
        return np.array(points_list)
    
    def compute_weld_dimensions(self, points):
        """
        Compute weld width and height from pointcloud using RANSAC + DBSCAN
        Returns: (width, height) in mm
        """
        if len(points) < 10:
            return 0.0, 0.0
        
        try:
            # Convert to mm
            points = points * 1000.0
            
            # Extract coordinates
            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]

            # Fit base plane using RANSAC
            y_reshape = y.reshape(-1, 1)
            ransac = RANSACRegressor(min_samples=20, residual_threshold=0.2)
            ransac.fit(y_reshape, z)
            z_pred = ransac.predict(y_reshape)
            z_base = np.median(z_pred)
            
            # Get inlier mask (base surface)
            inlier_mask = ransac.inlier_mask_

            # Get outlier points (potential weld)
            outlier_indices = np.where(~inlier_mask)[0]
            
            if len(outlier_indices) < 2:
                return 0.0, 0.0
            
            # Cluster outlier points to find main weld bead
            outlier_points = points[outlier_indices]
            
            # Use DBSCAN to find dense clusters
            clustering = DBSCAN(eps=0.5, min_samples=5).fit(outlier_points[:, :2])
            labels = clustering.labels_
            
            # Find the largest cluster (ignore noise points labeled as -1)
            unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
            
            if len(unique_labels) == 0:
                # No valid clusters found, use all outliers
                weld_indices = outlier_indices
            else:
                # Get the largest cluster
                largest_cluster_label = unique_labels[np.argmax(counts)]
                cluster_mask = labels == largest_cluster_label
                weld_indices = outlier_indices[cluster_mask]
            
            # Create weld mask
            weld_mask = np.zeros(len(points), dtype=bool)
            weld_mask[weld_indices] = True

            # Find weld top (height) from main cluster only
            z_relative = z - z_base
            z_relative_weld = z_relative[weld_mask]
            
            if len(z_relative_weld) == 0:
                return 0.0, 0.0
            
            height_raw = np.max(z_relative_weld)
            
            # Check if height is below threshold - no weld detected
            if height_raw < 0.5:
                # self.get_logger().log('No weld detected (height < 0.5mm)')
                return 0.0, 0.0
            
            # Estimate weld width using only main cluster points above half-height threshold
            y_weld = y[weld_mask]
            z_relative_weld_full = z_relative[weld_mask]
            
            # Filter to points above 5% of peak height
            half_height = 0.05 * height_raw
            above_threshold = z_relative_weld_full >= half_height
            
            if np.sum(above_threshold) < 2:
                # Fallback: use all weld points
                y_filtered = y_weld
            else:
                y_filtered = y_weld[above_threshold]
            
            # Width is the extent of filtered weld points
            width_raw = np.max(y_filtered) - np.min(y_filtered)
            
            # Apply moving average filter
            self._width_history.append(width_raw)
            self._height_history.append(height_raw)
            
            width = np.mean(self._width_history)
            height = np.mean(self._height_history)

            # self.get_logger().info(f'Weld Width: {width:.1f}mm, Height: {height:.1f}mm '
            #                      f'(Base: {np.sum(inlier_mask)}, Weld: {np.sum(weld_mask)}, '
            #                      f'Noise: {len(points) - np.sum(inlier_mask) - np.sum(weld_mask)})')
            
            return float(width), float(height)
            
        except Exception as e:
            self.get_logger().error(f'Error computing weld dimensions: {e}')
            return 0.0, 0.0


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
