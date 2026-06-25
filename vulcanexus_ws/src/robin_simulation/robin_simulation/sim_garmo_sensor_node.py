"""Simulated Garmo laser profilometer — relay from Gazebo GPU lidar.

Drop-in replacement for garmo_sensor_node. Relays Gazebo lidar pointcloud
to robin/pointcloud, transforming from sim_lidar_frame to laser_frame.

Implemented as a LifecycleNode to match the real hardware sensor_node so
the lifecycle_manager_sensor can manage it correctly:
  - on_configure: create publisher, TF infra, attempt TF lookup (with retry)
  - on_activate:  subscribe to /garmo_laser/points
  - on_deactivate: unsubscribe
  - on_cleanup/on_shutdown: destroy resources
"""

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
import tf2_ros


def _quat_to_matrix(x, y, z, w):
    """Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


class SimGarmoSensor(LifecycleNode):

    def __init__(self):
        super().__init__('garmo_sensor_node')
        self._pub = None
        self._sub = None
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_retry_timer = None
        self._rot = None    # 3×3 rotation sim_lidar_frame → laser_frame
        self._trans = None  # (3,) translation

    # -- Lifecycle callbacks --------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._pub = self.create_publisher(
            PointCloud2, 'robin/pointcloud', qos_profile_sensor_data)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.get_logger().info('Configured: Gazebo laser profiler relay')

        # Attempt TF lookup; schedule retry timer if not yet available
        self._try_lookup_tf()
        if self._rot is None:
            self._tf_retry_timer = self.create_timer(1.0, self._tf_retry_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(
            PointCloud2, '/garmo_laser/points',
            self._on_gz_pointcloud, qos_profile_sensor_data)
        self.get_logger().info('Activated: relaying Gazebo lidar')
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        self._cancel_tf_retry()
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._cancel_tf_retry()
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        self._tf_listener = None
        self._tf_buffer = None
        self._rot = None
        self._trans = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._cancel_tf_retry()
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    # -- TF helpers -----------------------------------------------------------

    def _try_lookup_tf(self):
        """Attempt a non-blocking TF lookup; sets _rot/_trans on success."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'laser_frame', 'sim_lidar_frame',
                Time(), timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            self._trans = np.array([t.x, t.y, t.z], dtype=np.float64)
            self._rot = _quat_to_matrix(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'Transform sim_lidar_frame→laser_frame acquired: '
                f't=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}]')
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed (will retry): {e}')

    def _tf_retry_cb(self):
        """Timer callback: retry TF lookup until it succeeds."""
        self._try_lookup_tf()
        if self._rot is not None:
            self._cancel_tf_retry()

    def _cancel_tf_retry(self):
        if self._tf_retry_timer is not None:
            self._tf_retry_timer.cancel()
            self.destroy_timer(self._tf_retry_timer)
            self._tf_retry_timer = None

    # -- Pointcloud relay -----------------------------------------------------

    def _on_gz_pointcloud(self, msg: PointCloud2):
        self._pub.publish(self._transform_cloud(msg))

    def _transform_cloud(self, msg: PointCloud2) -> PointCloud2:
        """Transform pointcloud from sim_lidar_frame → laser_frame."""
        n = msg.width * msg.height
        if n == 0 or self._rot is None:
            msg.header.frame_id = 'laser_frame'
            return msg

        ps = msg.point_step
        offsets = {f.name: f.offset
                   for f in msg.fields if f.name in ('x', 'y', 'z')}
        if len(offsets) < 3:
            msg.header.frame_id = 'laser_frame'
            return msg

        ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps).copy()

        # Extract xyz columns as float32
        x = raw[:, ox:ox+4].copy().view(np.float32).flatten()
        y = raw[:, oy:oy+4].copy().view(np.float32).flatten()
        z = raw[:, oz:oz+4].copy().view(np.float32).flatten()

        xyz = np.column_stack([x, y, z]).astype(np.float64)
        xyz_t = (self._rot @ xyz.T).T + self._trans

        # Write transformed xyz back into raw buffer
        raw[:, ox:ox+4] = xyz_t[:, 0:1].astype(np.float32).view(np.uint8)
        raw[:, oy:oy+4] = xyz_t[:, 1:2].astype(np.float32).view(np.uint8)
        raw[:, oz:oz+4] = xyz_t[:, 2:3].astype(np.float32).view(np.uint8)

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'laser_frame'
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = ps
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = raw.tobytes()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = SimGarmoSensor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
