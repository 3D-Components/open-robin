import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import numpy as np
import socket
import threading
import time
import math
import tf2_ros


def _resolve_scale(units):
    # 1 raw unit = 10 µm
    if isinstance(units, (int, float)):
        return float(units)
    if not units:
        return 1.0
    u = str(units).lower()
    if u in ("raw",):
        return 1.0
    if u in ("um", "µm", "micrometer", "micrometers"):
        return 10.0               # raw -> µm
    if u in ("mm", "millimeter", "millimeters"):
        return 0.01               # raw -> mm
    if u in ("m", "meter", "meters"):
        return 1e-5               # raw -> m
    raise ValueError(f"Unsupported units: {units}")


class SensorPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('robin_sensor_publisher')

        # Parameters (can be overridden via ros2 param)
        self.declare_parameter('sensor_ip', '192.168.1.212')
        self.declare_parameter('data_port', 66)
        self.declare_parameter('frame_id', 'garmo_laser_frame')                # default to URDF frame name
        self.declare_parameter('parent_frame', 'world')                        # default to URDF root link
        self.declare_parameter('frame_xyz', [0.0, 0.0, 0.0])        # NEW: sensor translation (meters by default)
        self.declare_parameter('frame_rpy', [0.0, 0.0, 0.0])        # NEW: sensor rotation (radians)
        self.declare_parameter('tf_broadcast_hz', 42.0)                # NEW: TF broadcast rate (Hz)
        self.declare_parameter('topic', '/robin/pointcloud')
        self.declare_parameter('units', 'm')  # raw/um/mm/m
        self.declare_parameter('reconnect_interval', 2.0)
        # NEW: optional toggle to log each publish (debug level)
        self.declare_parameter('log_publish_debug', False)

        self.sensor_ip = self.get_parameter('sensor_ip').value
        self.data_port = int(self.get_parameter('data_port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        frame_xyz_param = self.get_parameter('frame_xyz').value
        frame_rpy_param = self.get_parameter('frame_rpy').value
        self.tf_broadcast_hz = float(self.get_parameter('tf_broadcast_hz').value)
        topic = self.get_parameter('topic').value
        self.units = self.get_parameter('units').value
        self._xyz_cols = [0, 1, 2]
        self._frame_col = 4
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.log_publish_debug = bool(self.get_parameter('log_publish_debug').value)

        # normalize pose params (moved into helper for stricter validation)
        self.frame_xyz = self._normalize_vec3(frame_xyz_param, "frame_xyz")
        self.frame_rpy = self._normalize_vec3(frame_rpy_param, "frame_rpy")

        self.scale = _resolve_scale(self.units)

        # Publisher
        self._pub = self.create_publisher(PointCloud2, topic, 10)

        # TF broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # timer to broadcast transform periodically
        tf_period = max(0.01, 1.0 / max(0.1, self.tf_broadcast_hz))
        self._tf_timer = self.create_timer(tf_period, self._broadcast_transform)

        # Reader thread control
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f"Sensor data publisher started: {self.sensor_ip}:{self.data_port} -> {topic} "
                               f"frame='{self.frame_id}' parent='{self.parent_frame}' units={self.units}")

    def _normalize_vec3(self, seq, name):
        """Ensure a sequence of 3 numeric values; fallback to (0,0,0) with warning."""
        try:
            if not isinstance(seq, (list, tuple)):
                raise ValueError("Not a sequence")
            if len(seq) != 3:
                raise ValueError(f"Expected length 3, got {len(seq)}")
            vals = tuple(float(x) for x in seq)
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                raise ValueError("Values contain NaN/Inf")
            return vals
        except Exception as e:
            self.get_logger().warning(f"Parameter {name} invalid ({e}); using (0,0,0)")
            return (0.0, 0.0, 0.0)

    def _reader_loop(self):
        text_buffer = ""
        current_points = []
        current_frame = None

        while not self._stop_event.is_set():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2.0)
                    s.connect((self.sensor_ip, self.data_port))
                    s.settimeout(1.0)
                    self.get_logger().info("Connected to sensor data stream")
                    while not self._stop_event.is_set():
                        try:
                            chunk = s.recv(4096)
                            if not chunk:
                                # connection closed by peer
                                raise OSError("Socket closed")
                        except socket.timeout:
                            continue
                        except OSError:
                            break

                        try:
                            text_buffer += chunk.decode("ascii", errors="ignore")
                        except Exception:
                            continue

                        # Process complete lines
                        while "\n" in text_buffer:
                            line, text_buffer = text_buffer.split("\n", 1)
                            line = line.strip("\r").strip()
                            if not line:
                                continue

                            parts = line.split()
                            needed_len = max(max(self._xyz_cols), self._frame_col) + 1
                            if len(parts) < needed_len:
                                continue

                            try:
                                values = [int(p) for p in parts]
                            except ValueError:
                                continue

                            frame_no = values[self._frame_col]
                            if current_frame is None:
                                current_frame = frame_no

                            # New frame detected -> publish accumulated points
                            if current_points and frame_no != current_frame:
                                self._publish_pointcloud(current_points)
                                current_points = []
                                current_frame = frame_no

                            try:
                                px = values[self._xyz_cols[0]] * self.scale
                                py = values[self._xyz_cols[1]] * self.scale
                                pz = values[self._xyz_cols[2]] * self.scale
                                # Basic sanity (reject absurd magnitudes to avoid RViz crashes)
                                if not (-1e4 < px < 1e4 and -1e4 < py < 1e4 and -1e4 < pz < 1e4):
                                    continue
                                current_points.append((px, py, pz))
                            except Exception:
                                pass

                    # On connection break publish any accumulated points
                    if current_points:
                        self._publish_pointcloud(current_points)
                        current_points = []
                        current_frame = None

            except Exception as e:
                if self._stop_event.is_set():
                    break
                self.get_logger().warning(f"Data stream connection lost: {e}. Reconnecting in {self.reconnect_interval}s")
                # small sleep before reconnecting
                time.sleep(self.reconnect_interval)
                continue

    def _publish_pointcloud(self, points):
        if not points:
            return
        try:
            pts = np.asarray(points, dtype=np.float32)
        except Exception as e:
            self.get_logger().warning(f"Failed to build point array: {e}")
            return
        if pts.ndim != 2 or pts.shape[1] < 3 or pts.size == 0:
            return

        n_points = pts.shape[0]
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n_points
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * n_points
        msg.data = pts.tobytes()

        self._pub.publish(msg)
        if self.log_publish_debug and self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().debug(f"Published PointCloud2 with {n_points} points")

    def _broadcast_transform(self):
        """
        Broadcast a transform parent_frame -> frame_id using configured xyz/rpy.
        This provides the TF needed by RViz to transform the PointCloud2 into other frames.
        """
        # Skip broadcasting if frames identical (would be invalid TF)
        if self.parent_frame == self.frame_id:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id

        # translation
        tx, ty, tz = self.frame_xyz
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)

        # rotation: convert RPY (roll, pitch, yaw) to quaternion
        roll, pitch, yaw = self.frame_rpy
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        try:
            self._tf_broadcaster.sendTransform(t)
        except Exception as e:
            # avoid noisy logging on transient broadcaster errors
            self.get_logger().debug(f"TF sendTransform error: {e}")

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

    def destroy_node(self):
        # Signal thread to stop and wait
        self._stop_event.set()
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        # cancel TF timer
        try:
            if hasattr(self, "_tf_timer"):
                self._tf_timer.cancel()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
