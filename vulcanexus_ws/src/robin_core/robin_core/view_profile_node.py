#!/usr/bin/env python3
"""Live weld cross-section viewer.

Subscribes to /robin/pointcloud and /robin/weld_dimensions, runs the
same processing pipeline as process_data_node.py, and displays a live
matplotlib plot showing:

  - Raw Z profile (grey dots)
  - Smoothed Z profile (blue line)
  - RANSAC base plane (green dashed)
  - Bead threshold line (red dashed)
  - Detected toe positions (red vertical lines)
  - Width / height annotation

Usage (inside vulcanexus container):
  source /workspace/ros2_packages/ws_setup.sh
  python3 /workspace/ros2_packages/src/robin_core/scripts/view_profile.py

Press 'q' or close the window to exit.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from robin_interfaces.msg import BeadGeometry
import numpy as np
import numpy.lib.recfunctions as rfn
import sensor_msgs_py.point_cloud2 as pc2
from robin_core.sensor.profile_analysis import analyse_weld_profile
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import time


class ProfileViewer(Node):
    def __init__(self):
        super().__init__("profile_viewer")

        self.declare_parameter("input_topic", "/robin/pointcloud")
        self.declare_parameter("geometry_topic", "/robin/weld_dimensions")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        geometry_topic = self.get_parameter("geometry_topic").get_parameter_value().string_value

        # Latest published geometry (from the real processor node)
        self._last_geom = None
        # Throttle redraws to ~4 Hz
        self._last_draw_time = 0.0
        self._min_draw_interval = 0.25  # seconds (= 4 Hz)

        self.create_subscription(
            PointCloud2, input_topic, self._pc_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            BeadGeometry, geometry_topic, self._geom_cb, qos_profile_sensor_data
        )

        # --- matplotlib setup ---
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(10, 5))
        self._fig.canvas.manager.set_window_title("Weld Cross-Section Viewer")
        self._ax.set_xlabel("Y  (mm)")
        self._ax.set_ylabel("Z  (mm)")
        self._ax.set_title("Waiting for pointcloud…")
        self._fig.tight_layout()
        self._fig.show()

        # Timer to pump the matplotlib event loop
        self.create_timer(0.05, self._pump_gui)

        self.get_logger().info(
            f"Profile viewer started — listening on {input_topic}"
        )

    # ------------------------------------------------------------------
    def _geom_cb(self, msg: BeadGeometry):
        self._last_geom = msg

    # ------------------------------------------------------------------
    def _pump_gui(self):
        """Keep the matplotlib window responsive."""
        try:
            self._fig.canvas.flush_events()
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _pc_cb(self, msg: PointCloud2):
        # Throttle redraws
        now = time.monotonic()
        if now - self._last_draw_time < self._min_draw_interval:
            return
        self._last_draw_time = now

        structured = np.array(
            list(pc2.read_points(msg, skip_nans=True,
                                 field_names=("x", "y", "z"))))
        if structured.size == 0:
            return
        points = rfn.structured_to_unstructured(structured).astype(np.float64)
        if len(points) < 20:
            return

        points_mm = points * 1000.0
        sort_idx = np.argsort(points_mm[:, 1])
        points_mm = points_mm[sort_idx]

        r = analyse_weld_profile(points_mm)
        y = r['y']
        z = r['z']
        z_base = r['z_base']
        base_mask = r['base_mask']
        bead_threshold = r['bead_threshold']
        z_relative = r['z_relative']
        starts = r['starts']
        ends = r['ends']
        si = r['start_idx']
        ei = r['end_idx']
        y_left = r['y_left']
        y_right = r['y_right']
        width = r['width']
        height = r['height']
        toe_angle = r['toe_angle_rad']

        # --- draw --------------------------------------------------------
        ax = self._ax
        ax.cla()

        # Raw points
        ax.plot(y, z, ".", color="0.70", markersize=2, label="raw")
        # Base plate inlier points (green circles)
        ax.plot(y[base_mask], z[base_mask], "o", color="tab:green",
                markersize=3, alpha=0.4, label="base inliers")
        # Smoothed profile
        ax.plot(y, z, "-", color="tab:blue", linewidth=1.2,
                label="profile")
        # Base plane
        ax.plot(y, z_base, "--", color="tab:green", linewidth=1,
                label="base plane")
        # Threshold line (relative to base)
        ax.plot(y, z_base + bead_threshold, "--", color="tab:red",
                linewidth=0.8, alpha=0.6,
                label=f"threshold ({bead_threshold:.2f} mm)")

        # Shade non-target bead segments (if any)
        if len(starts) > 0 and si is not None:
            for i in range(len(starts)):
                seg_si, seg_ei = starts[i], ends[i] - 1
                if seg_si == si and seg_ei == ei:
                    continue  # skip the target bead
                if (y[seg_ei] - y[seg_si]) >= 2.0:
                    lbl = "adjacent bead" if i == 0 else None
                    ax.axvspan(y[seg_si], y[seg_ei],
                               alpha=0.10, color="grey", label=lbl)

        # Toe markers and dimension annotations
        title_extra = ""

        if y_left is not None and y_right is not None:
            # Reference geometry for positioning arrows
            peak_idx = si + np.argmax(z_relative[si:ei + 1])
            peak_y = y[peak_idx]
            peak_z_base = z_base[peak_idx]
            bead_center_y = (y_left + y_right) / 2.0
            z_arrow_base = np.min(z_base) - 0.15

            ax.axvline(y_left, color="tab:red", linewidth=1.5,
                       linestyle="-", alpha=0.8)
            ax.axvline(y_right, color="tab:red", linewidth=1.5,
                       linestyle="-", alpha=0.8)
            # Width arrow
            ax.annotate("", xy=(y_right, z_arrow_base),
                        xytext=(y_left, z_arrow_base),
                        arrowprops=dict(arrowstyle="<->", color="tab:red",
                                        lw=1.5))
            ax.text(bead_center_y, z_arrow_base - 0.12,
                    f"W = {width:.2f} mm", ha="center", va="top",
                    fontsize=10, color="tab:red", fontweight="bold")
            # Height arrow
            ax.annotate("", xy=(peak_y, peak_z_base + height),
                        xytext=(peak_y, peak_z_base),
                        arrowprops=dict(arrowstyle="<->", color="tab:purple",
                                        lw=1.5))
            ax.text(peak_y + 0.5, peak_z_base + height / 2,
                    f"H = {height:.2f} mm", ha="left", va="center",
                    fontsize=10, color="tab:purple", fontweight="bold")
            title_extra = f"  —  W={width:.2f}  H={height:.2f} mm"
            if toe_angle is not None:
                title_extra += f"  θ={toe_angle:.2f} rad"

        ax.set_xlabel("Y  (mm)")
        ax.set_ylabel("Z  (mm)")
        ax.set_xlim(-15, 15)
        ax.set_ylim(-5, 15)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"Weld Cross-Section{title_extra}", fontsize=11)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        self._fig.tight_layout()
        self._fig.canvas.draw_idle()


def main():
    rclpy.init()
    node = ProfileViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
