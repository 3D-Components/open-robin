import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import numpy as np
import select
import socket

import tf2_ros
from scipy.spatial.transform import Rotation


CSV_START_TRACK = b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x01\xca\xca'
CSV_END_TRACK =   b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x00\xca\xca'

_FRAME_COL = 4
_MIN_FIELDS = _FRAME_COL + 1
_SCALE = 1e-5  # raw sensor units → meters
_BOUNDS = 1e4
_RECV_SIZE = 65536


class GarmoSensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('garmo_sensor_node')

        self.declare_parameters('', [
            ('sensor_ip', '192.168.1.212'),
            ('ctrl_port', 5020),
            ('data_port', 66),
            ('fps', 42),
            ('frame_id', 'garmo_laser_frame'),
            ('parent_frame', 'tool0'),
            ('frame_xyz', [0.0, 0.0, 0.0]),
            ('frame_rpy', [0.0, 0.0, 0.0]),
            ('smoothing_window', 51),
            ('smoothing_sigma_r', 0.001),
        ])

        self._sock = None
        self._recv_buf = b''
        self._current_points = []
        self._current_frame = None
        self._poll_timer = None

        # Dedicated callback group so the data timer gets its own executor
        # thread and never blocks lifecycle service callbacks.
        self._data_cb_group = MutuallyExclusiveCallbackGroup()

    # -- Static helpers -----------------------------------------------------

    @staticmethod
    def build_set_fps_command(fps_value: float) -> bytes:
        """Build the CSV set-fps command bytes. Period (ms) = round(1000/fps)."""
        if fps_value <= 0:
            raise ValueError("fps must be > 0")
        period_ms = max(1, min(0xFFFF, round(1000.0 / fps_value)))
        return (b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9'
                + period_ms.to_bytes(2, 'big') + b'\xca\xca')

    @staticmethod
    def send_command(ip, port: int, cmd_bytes: bytes,
                     timeout: float = 1.0) -> bytes | None:
        """Send byte-array command to sensor control port and return reply."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((ip, int(port)))
            s.sendall(cmd_bytes)
            try:
                return s.recv(4096)
            except socket.timeout:
                return None

    @staticmethod
    def _bilateral_filter_1d(z, window, sigma_r):
        """Edge-preserving 1D bilateral filter on z values.

        Smooths flat regions while preserving sharp transitions (e.g. bead
        walls) by weighting neighbours with both spatial and value similarity.
        """
        half_w = window // 2
        sigma_s = half_w / 3.0

        z_pad = np.pad(z, half_w, mode='reflect')
        offsets = np.arange(-half_w, half_w + 1, dtype=np.float64)
        spatial_w = np.exp(-0.5 * (offsets / sigma_s) ** 2)

        windows = np.lib.stride_tricks.sliding_window_view(z_pad, window)
        range_w = np.exp(-0.5 * ((windows - z[:, None]) / sigma_r) ** 2)
        weights = spatial_w[None, :] * range_w
        return np.sum(weights * windows, axis=1) / np.sum(weights, axis=1)

    # -- Lifecycle callbacks ------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.sensor_ip = self.get_parameter('sensor_ip').value
        self.ctrl_port = int(self.get_parameter('ctrl_port').value)
        self.data_port = int(self.get_parameter('data_port').value)
        self.fps = int(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.frame_xyz = np.array(self.get_parameter('frame_xyz').value, dtype=np.float64)
        self.frame_rpy = np.array(self.get_parameter('frame_rpy').value, dtype=np.float64)
        self.smoothing_window = int(self.get_parameter('smoothing_window').value)
        self.smoothing_sigma_r = float(self.get_parameter('smoothing_sigma_r').value)

        self._data_pub = self.create_lifecycle_publisher(
            PointCloud2, 'robin/pointcloud', qos_profile_sensor_data)
        
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transform()

        try:
            reply = self.send_command(
                self.sensor_ip, self.ctrl_port,
                self.build_set_fps_command(self.fps))
            self.get_logger().info(
                f"Set sensor FPS to {self.fps}"
                f" (reply: {reply.hex() if reply else 'none'})")
        except Exception as e:
            self.get_logger().warning(f"Failed to set sensor FPS: {e}")

        self.get_logger().info(
            f"Configured: {self.sensor_ip} "
            f"ctrl={self.ctrl_port} data={self.data_port}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        try:
            self.send_command(self.sensor_ip, self.ctrl_port, CSV_START_TRACK)
        except Exception as e:
            self.get_logger().error(f"Failed to start tracking: {e}")
            return TransitionCallbackReturn.FAILURE

        # Open persistent data socket (non-blocking for timer-driven polling)
        try:
            self._sock = socket.create_connection(
                (self.sensor_ip, self.data_port), timeout=2.0)
            self._sock.setblocking(False)
        except Exception as e:
            self.get_logger().error(f"Failed to connect data port: {e}")
            return TransitionCallbackReturn.FAILURE

        self._recv_buf = b''
        self._current_points = []
        self._current_frame = None

        # Poll at 2× sensor FPS so we drain data faster than it arrives.
        # Runs in its own callback group / executor thread.
        self._poll_timer = self.create_timer(
            0.5 / self.fps, self._poll_socket,
            callback_group=self._data_cb_group)

        self.get_logger().info('Activated — sensor stream started')
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._stop_streaming()
        if self._current_points:
            self._publish_pointcloud(self._current_points)
            self._current_points = []
        self._current_frame = None
        self.get_logger().info('Deactivated — sensor stream stopped')
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._data_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._stop_streaming()
        return TransitionCallbackReturn.SUCCESS

    def _stop_streaming(self):
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None
        self._close_socket()
        try:
            self.send_command(self.sensor_ip, self.ctrl_port, CSV_END_TRACK)
        except Exception:
            pass

    # -- Timer-driven socket reader -----------------------------------------

    def _poll_socket(self):
        """Timer callback: drain available data, parse lines, publish frames."""
        if self._sock is None:
            return

        # Read all available data without blocking
        try:
            while select.select([self._sock], [], [], 0)[0]:
                chunk = self._sock.recv(_RECV_SIZE)
                if not chunk:  # peer closed
                    self.get_logger().warning("Sensor data stream closed by peer")
                    self._close_socket()
                    return
                self._recv_buf += chunk
        except (OSError, socket.error) as e:
            self.get_logger().warning(f"Socket recv error: {e}")
            self._close_socket()
            return

        # Split into complete lines; keep any trailing partial line in buffer
        *lines, self._recv_buf = self._recv_buf.split(b'\n')

        for raw_line in lines:
            parts = raw_line.split()
            if len(parts) < _MIN_FIELDS:
                continue

            try:
                frame_no = int(parts[_FRAME_COL])
            except ValueError:
                continue

            # Frame boundary → publish accumulated points
            if self._current_frame is not None and frame_no != self._current_frame:
                self._publish_pointcloud(self._current_points)
                self._current_points = []

            self._current_frame = frame_no

            try:
                xyz = tuple(float(parts[i]) * _SCALE for i in range(3))
            except ValueError:
                continue

            if all(-_BOUNDS < v < _BOUNDS for v in xyz):
                self._current_points.append(xyz)

    def _close_socket(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._recv_buf = b''

    def _publish_pointcloud(self, points):
        if not points:
            return

        pts = np.array(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            return

        # Sort by Y, apply edge-preserving bilateral filter to Z
        pts = pts[np.argsort(pts[:, 1])]
        win = min(self.smoothing_window, len(pts))
        if win % 2 == 0:
            win -= 1
        if win >= 3 and self.smoothing_sigma_r > 0:
            pts[:, 2] = self._bilateral_filter_1d(
                pts[:, 2], win, self.smoothing_sigma_r)

        n = pts.shape[0]
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name=c, offset=i * 4,
                       datatype=PointField.FLOAT32, count=1)
            for i, c in enumerate('xyz')
        ]
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.data = pts[:, :3].tobytes()

        self._data_pub.publish(msg)

    def _publish_static_transform(self):
        """Broadcast static transform parent_frame -> frame_id."""
        if self.parent_frame == self.frame_id:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id

        tr = t.transform
        tr.translation.x, tr.translation.y, tr.translation.z = self.frame_xyz.tolist()
        q = Rotation.from_euler('xyz', self.frame_rpy).as_quat()
        tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w = q.tolist()

        try:
            self._tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warning(f"Static TF sendTransform error: {e}")

    def destroy_node(self):
        self._close_socket()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GarmoSensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
