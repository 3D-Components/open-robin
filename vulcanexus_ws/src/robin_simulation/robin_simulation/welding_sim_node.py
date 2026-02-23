import rclpy
from rclpy.node import Node
import numpy as np
import math

# Import our simulation logic (no changes needed to this file)
from . import welding_simulator_lib as sim_lib

# Import ROS 2 messages
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

# Import our NEW custom messages
from robin_interfaces.msg import WelderData, BeadGeometry

class WeldingSimNode(Node):
    def __init__(self):
        super().__init__('welding_sim_node')
        self.get_logger().info('Hardware Simulator Node started.')

        # --- Parameters (Welding Inputs) ---
        self.declare_parameter('process_id', 'default_process')
        self.process_id = self.get_parameter('process_id').get_parameter_value().string_value
        
        self.wire_feed = self.declare_parameter('wire_feed_rate_m_min', 8.0).value
        self.current = self.declare_parameter('current_a', 200.0).value
        self.voltage = self.declare_parameter('voltage_v', 22.0).value
        
        # --- Frequencies (to match real hardware) ---
        self.sim_rate_hz = 200.0  # How fast to run the internal physics
        self.welder_rate_hz = 10.0  # Fronius
        self.profiler_rate_hz = 42.0  # Garline (preprocessed)
        self.robot_rate_hz = 100.0  # UR10e (Guessing 100Hz for [Z])

        # --- Publishers (Mirroring the real topics) ---
        self.welder_pub = self.create_publisher(
            WelderData, '/welder/data', 10)
        
        self.robot_pub = self.create_publisher(
            TwistStamped, '/tool_velocity', 10)
            
        self.geometry_pub = self.create_publisher(
            BeadGeometry, '/welding/bead_geometry', 10) # The "dedicated topic"
        
        # --- Simulation State ---
        self.sim_params = sim_lib.WeldParams()
        self.rng = np.random.default_rng(self.sim_params.seed)
        self.robot_frame_id = "welding_torch"
        self.weld_bead_frame_id = "weld_bead"

        # Pre-calculate constants
        r_mm = self.sim_params.wire_diameter_mm / 2.0
        self.A_wire_mm2 = math.pi * r_mm * r_mm
        self.v_wire_mm_s = self.wire_feed * 1000.0 / 60.0
        self.drift = 0.0

        # Internal state variables, updated by the simulation
        self.current_height_mm = 0.0
        self.current_width_mm = 0.0
        self.current_travel_speed_mm_s = 0.0
        self.current_area_mm2 = 0.0

        # --- Timers ---
        # A fast timer to run the physics simulation
        self.sim_timer = self.create_timer(
            1.0 / self.sim_rate_hz, self.update_simulation_state)
        
        # Separate timers for each "hardware" publisher
        self.welder_timer = self.create_timer(
            1.0 / self.welder_rate_hz, self.publish_welder_data)
            
        self.robot_timer = self.create_timer(
            1.0 / self.robot_rate_hz, self.publish_robot_data)
            
        self.geometry_timer = self.create_timer(
            1.0 / self.profiler_rate_hz, self.publish_geometry_data)

        self.get_logger().info(f'Starting simulation for process: {self.process_id}')

    def update_simulation_state(self):
        """
        This is the core physics simulation.
        It runs at a high frequency to update the "state of the world".
        """
        try:
            p = self.sim_params
            
            # This is the logic from your simulate_beads() loop
            eta_dep = sim_lib._truncnorm(p.eta_dep_mean, p.eta_dep_sd, p.eta_bounds[0], p.eta_bounds[1], rng=self.rng)
            eta_arc = sim_lib._truncnorm(p.eta_arc_mean, p.eta_arc_sd, p.eta_bounds[0], p.eta_bounds[1], rng=self.rng)

            self.drift += self.rng.normal(0.0, p.drift_sd_mm_s)
            vt = (
                p.v0_mm_s
                + p.k_I_mm_s_per_A * self.current
                + p.k_WFS_mm_s_per_mpm * self.wire_feed
                + self.rng.normal(0.0, p.vt_sd_mm_s)
                + self.drift
            )
            vt = max(vt, p.vt_min_mm_s)
            
            dep_rate_mm3_s = eta_dep * self.A_wire_mm2 * self.v_wire_mm_s
            A_bead_mm2 = dep_rate_mm3_s / vt
            heat_input_J_per_mm = (eta_arc * self.voltage * self.current) / vt
            width_mm = p.kw * (heat_input_J_per_mm**p.a_exp)
            width_mm = max(width_mm, 0.5)
            height_mm = (4.0 * A_bead_mm2) / (math.pi * width_mm)

            # Update the shared state variables
            self.current_height_mm = height_mm
            self.current_width_mm = width_mm
            self.current_travel_speed_mm_s = vt
            self.current_area_mm2 = A_bead_mm2

        except Exception as e:
            self.get_logger().error(f'Error in simulation state update: {e}')

    def publish_welder_data(self):
        """Publishes the (low-frequency) welder parameters."""
        msg = WelderData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_frame_id
        msg.voltage_v = self.voltage
        msg.current_a = self.current
        msg.wire_feed_speed_mpm = self.wire_feed
        self.welder_pub.publish(msg)

    def publish_robot_data(self):
        """Publishes the (high-frequency) robot tool speed."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_frame_id
        
        # Convert travel speed from mm/s to m/s for ROS standard
        msg.twist.linear.x = self.current_travel_speed_mm_s / 1000.0
        
        self.robot_pub.publish(msg)
            
    def publish_geometry_data(self):
        """Publishes the (medium-frequency) preprocessed bead geometry."""
        msg = BeadGeometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.weld_bead_frame_id
        msg.height_mm = self.current_height_mm
        msg.width_mm = self.current_width_mm
        msg.cross_sectional_area_mm2 = self.current_area_mm2
        self.geometry_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WeldingSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()