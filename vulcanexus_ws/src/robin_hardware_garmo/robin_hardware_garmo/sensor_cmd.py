from robin_interfaces.srv import SensorCommand

import rclpy
from rclpy.node import Node
import socket
import typing

CSV_START_TRACK =   b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x01\xca\xca'
CSV_END_TRACK =     b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x00\xca\xca'
CSV_SET_FPS_HIGH =  b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9\x00\x18\xca\xca'
CSV_SET_FPS_LOW =   b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9\x03\xE8\xca\xca'
CSV_SET_JOINT =     b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x10\x00\x01\xca\xca'

class SensorCommandService(Node):

    def __init__(self):
        super().__init__('sensor_command_service')

        # Declare configurable parameters (can be overridden at runtime or via launch)
        self.declare_parameter('sensor_ip', '192.168.1.212')
        self.declare_parameter('ctrl_port', 5020)
        self.declare_parameter('data_port', 66)
        self.declare_parameter('fps', 42)  # frames-per-second as integer

        # activate: default workflow (set fps, set joint, start)
        self.srv = self.create_service(SensorCommand, 'profilometer_activate', self.activate_callback)
        # deactivate: stop sensor (end track)
        self.deactivate_srv = self.create_service(SensorCommand, 'profilometer_deactivate', self.deactivate_callback)

        # Log initial parameters
        self.get_logger().info(f"Sensor command service ready with sensor_ip={self.get_parameter('sensor_ip').value}, "
                               f"ctrl_port={self.get_parameter('ctrl_port').value}, data_port={self.get_parameter('data_port').value}, fps={self.get_parameter('fps').value}")

    def build_set_fps_command(self, fps_value: typing.Union[int, str]) -> bytes:
        """
        Build the CSV set-fps command bytes given an integer FPS.
        fps_value may be an int or numeric string. Frame period (ms) = round(1000 / fps).
        The period is encoded as two bytes big-endian and inserted into the command template.
        """
        try:
            fps_int = int(fps_value)
        except Exception:
            raise ValueError(f"Invalid fps value: {fps_value}")

        if fps_int <= 0:
            raise ValueError("fps must be > 0")

        # compute frame period in ms, clamp to 1..65535
        period_ms = int(round(1000.0 / float(fps_int)))
        period_ms = max(1, min(0xFFFF, period_ms))

        period_bytes = period_ms.to_bytes(2, byteorder='big')
        # template: prefix + command_code (0x00C9) + period bytes + trailer
        prefix = b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9'
        trailer = b'\xca\xca'
        return prefix + period_bytes + trailer

    def activate_callback(self, request, response):
        """
        Activate default workflow: set FPS (integer), set joint, start track.
        Request may override sensor_ip, ctrl_port, data_port, fps.
        """
        # Resolve parameters: prefer request attributes if present, else node params
        sensor_ip = getattr(request, 'sensor_ip', None) or self.get_parameter('sensor_ip').value
        ctrl_port = getattr(request, 'ctrl_port', None) or self.get_parameter('ctrl_port').value
        data_port = getattr(request, 'data_port', None) or self.get_parameter('data_port').value
        req_fps = getattr(request, 'fps', None)
        if req_fps is None:
            req_fps = self.get_parameter('fps').value

        # Normalize ports
        if isinstance(ctrl_port, str) and ctrl_port.isdigit():
            ctrl_port = int(ctrl_port)
        if isinstance(data_port, str) and data_port.isdigit():
            data_port = int(data_port)

        self.get_logger().info(f"Activate request: sensor_ip={sensor_ip} ctrl_port={ctrl_port} data_port={data_port} fps={req_fps}")

        # Build FPS command (require integer fps)
        try:
            fps_byte_cmd = self.build_set_fps_command(req_fps)
        except Exception as e:
            msg = f"Invalid fps value '{req_fps}': {e}"
            self.get_logger().error(msg)
            response.success = False
            if hasattr(response, 'message'):
                response.message = msg
            return response

        cmds_to_send = [fps_byte_cmd, CSV_SET_JOINT, CSV_START_TRACK]
        cmds_dict = {
            "Set FPS": fps_byte_cmd,
            "Set Joint": CSV_SET_JOINT,
            "Start Sensor": CSV_START_TRACK
        }

        overall_ok = True
        for cmd_name, cmd_bytes in cmds_dict.items():
            try:
                self.send_command(cmd_bytes, sensor_ip=sensor_ip, ctrl_port=int(ctrl_port))
                self.get_logger().info(f"Sensor Command Success: {cmd_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")
                overall_ok = False
                break

        response.success = bool(overall_ok)
        if hasattr(response, 'message'):
            response.message = "ok" if overall_ok else "error"
        return response

    def deactivate_callback(self, request, response):
        """
        Deactivate sensor: send end-track/stop command.
        Request may override sensor_ip and ctrl_port.
        """
        sensor_ip = getattr(request, 'sensor_ip', None) or self.get_parameter('sensor_ip').value
        ctrl_port = getattr(request, 'ctrl_port', None) or self.get_parameter('ctrl_port').value
        if isinstance(ctrl_port, str) and ctrl_port.isdigit():
            ctrl_port = int(ctrl_port)

        self.get_logger().info(f"Deactivate request: sensor_ip={sensor_ip} ctrl_port={ctrl_port}")

        try:
            self.send_command(CSV_END_TRACK, sensor_ip=sensor_ip, ctrl_port=int(ctrl_port))
            self.get_logger().info("Deactivated Sensor")
            response.success = True
            if hasattr(response, 'message'):
                response.message = "ok"
        except Exception as e:
            self.get_logger().error(f"Failed to send deactivate command: {e}")
            response.success = False
            if hasattr(response, 'message'):
                response.message = str(e)
        return response

    def send_command(self, cmd_bytes: bytes, sensor_ip: str = "192.168.1.100", ctrl_port: int = 5020, timeout: float = 1.0) -> typing.Optional[bytes]:
        """
        Send byte-array command to sensor control port and return reply bytes (or None if no reply).
        Similar behaviour to parse_sensor.send_command but integrated into the ROS node.
        """
        self.get_logger().info(f"Sending command to sensor {sensor_ip}:{ctrl_port} -> {cmd_bytes.hex()}")
        reply = None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((sensor_ip, int(ctrl_port)))
                s.sendall(cmd_bytes)
                try:
                    reply = s.recv(4096)
                except socket.timeout:
                    # no reply within timeout is acceptable for some commands
                    reply = None
        except Exception as e:
            self.get_logger().error(f"Socket error sending command: {e}")
            raise
        return reply

    def destroy_node(self):
        """
        Ensure the sensor is deactivated when this node is destroyed.
        """
        try:
            sensor_ip = self.get_parameter('sensor_ip').value
            ctrl_port = int(self.get_parameter('ctrl_port').value)
            self.get_logger().info(f"Shutting down sensor command service; sending deactivate to {sensor_ip}:{ctrl_port}")
            try:
                self.send_command(CSV_END_TRACK, sensor_ip=sensor_ip, ctrl_port=ctrl_port)
                self.get_logger().info("Sent CSV_END_TRACK on shutdown")
            except Exception as e:
                self.get_logger().warn(f"Failed to send CSV_END_TRACK on shutdown: {e}")
        except Exception as e:
            self.get_logger().warn(f"Could not read parameters to send deactivate: {e}")
        return super().destroy_node()


def main():
    rclpy.init()

    sensor_command_service = SensorCommandService()

    try:
        rclpy.spin(sensor_command_service)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure deactivate is attempted before shutdown
        try:
            sensor_command_service.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()