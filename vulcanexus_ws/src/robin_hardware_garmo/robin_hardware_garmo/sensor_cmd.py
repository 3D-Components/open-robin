from robin_interfaces.srv import SensorCommand

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import socket
import typing

CSV_START_TRACK =   b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x01\xca\xca'
CSV_END_TRACK =     b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x06\x00\x00\xca\xca'
CSV_SET_FPS_HIGH =  b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9\x00\x18\xca\xca'
CSV_SET_FPS_LOW =   b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\xC9\x03\xE8\xca\xca'
CSV_SET_JOINT =     b'\x00\x00\x00\x00\x02\x00\x00\x01\x00\x10\x00\x01\xca\xca'

class GarmoCommandNode(Node):

    def __init__(self):
        super().__init__('garmo_command_node')

        # Declare configurable parameters (can be overridden at runtime or via launch)
        self.declare_parameter('sensor_ip', '192.168.1.212')
        self.declare_parameter('ctrl_port', 5020)
        self.declare_parameter('fps', 42)  # frames-per-second as integer

        self.sensor_ip = self.get_parameter('sensor_ip').value
        self.ctrl_port = int(self.get_parameter('ctrl_port').value)
        self.fps = int(self.get_parameter('fps').value) 

        # activate: default workflow (set fps, set joint, start)
        self.srv = self.create_service(Trigger, 'profilometer_activate', self.activate_callback)
        # deactivate: stop sensor (end track)
        self.deactivate_srv = self.create_service(Trigger, 'profilometer_deactivate', self.deactivate_callback)

        # Log initial parameters
        self.get_logger().info(f"Sensor command service ready with sensor_ip={self.sensor_ip}, "
                               f"ctrl_port={self.ctrl_port}, fps={self.fps}")
        
        self.set_fps(self.fps)  # set initial FPS on startup

    @staticmethod
    def build_set_fps_command(fps_value: typing.Union[int, str]) -> bytes:
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
    
    def set_fps(self, fps_value: typing.Union[int, str]):
        """
        Send the set-fps command to the sensor with the given FPS value.
        """
        cmd_bytes = self.build_set_fps_command(fps_value)
        self.get_logger().info(f"Setting sensor FPS to {fps_value} -> sending command: {cmd_bytes.hex()}")
        reply = self.send_command(cmd_bytes)
        if reply is not None:
            self.get_logger().info(f"Received reply for set_fps command: {reply.hex()}")
        else:
            self.get_logger().info("No reply received for set_fps command (may be expected)")

    def activate_callback(self, request, response):
        """
        Activate default workflow: set FPS (integer), set joint, start track.
        """
        self.get_logger().info(f"Activating Sensor: sensor_ip={self.sensor_ip} ctrl_port={self.ctrl_port}")

        try:
            self.send_command(CSV_START_TRACK)
            response.success = True
            response.message = "Sensor Activated"
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Failed to activate sensor: {e}")

        return response

    def deactivate_callback(self, request, response):
        """
        Deactivate sensor: send end-track/stop command.
        """
        self.get_logger().info(f"Deactivate request: sensor_ip={self.sensor_ip} ctrl_port={self.ctrl_port}")

        try:
            self.send_command(CSV_END_TRACK)
            self.get_logger().info("Deactivated Sensor")
            response.success = True
            response.message = "Sensor Deactivated"
        except Exception as e:
            self.get_logger().error(f"Failed to send deactivate command: {e}")
            response.success = False
            response.message = str(e)
        return response

    def send_command(self, cmd_bytes: bytes, timeout: float = 1.0) -> typing.Optional[bytes]:
        """
        Send byte-array command to sensor control port and return reply bytes (or None if no reply).
        Similar behaviour to parse_sensor.send_command but integrated into the ROS node.
        """
        self.get_logger().info(f"Sending command to sensor {self.sensor_ip}:{self.ctrl_port} -> {cmd_bytes.hex()}")
        reply = None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((self.sensor_ip, int(self.ctrl_port)))
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
            sensor_ip = self.sensor_ip
            ctrl_port = self.ctrl_port
            self.get_logger().info(f"Shutting down sensor command service; sending deactivate to {sensor_ip}:{ctrl_port}")
            try:
                self.send_command(CSV_END_TRACK)
                self.get_logger().info("Sent CSV_END_TRACK on shutdown")
            except Exception as e:
                self.get_logger().warn(f"Failed to send CSV_END_TRACK on shutdown: {e}")
        except Exception as e:
            self.get_logger().warn(f"Could not read parameters to send deactivate: {e}")
        return super().destroy_node()


def main():
    rclpy.init()

    sensor_command_node = GarmoCommandNode()

    try:
        rclpy.spin(sensor_command_node)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure deactivate is attempted before shutdown
        try:
            sensor_command_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()