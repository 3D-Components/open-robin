#!/usr/bin/env python3
"""Publish plate visualization markers in RViz from persisted plates.json."""

import json
import os
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from robin_core.plate_geometry import normalize_corner_id, inward_to_world
from robin_core.bead_layout import compute_plate_occupancy_slots
from robin_core.bead_persistence import (
    get_occupied_slots,
    get_reserved_slots,
    load_completed_beads,
)


DEFAULT_MARGIN_M = 0.040
PLATE_THICKNESS = 0.006  # 6 mm — matches typical steel substrate thickness
SLOT_MARKER_HEIGHT = 0.004  # visual height for occupied slot markers
SLOT_MARKER_DIAMETER = 0.008  # visual diameter for occupied slot markers


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _point_to_array(p: Point) -> np.ndarray:
    return np.array([p.x, p.y, p.z])


def _safe_normalize(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else fallback


class PlateMarkersNode(Node):
    def __init__(self):
        super().__init__('plate_markers')
        self.declare_parameter('plates_config', '')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('beads_config', '')
        self.declare_parameter('default_spacing_x', 0.030)
        self.declare_parameter('default_spacing_y', 0.030)
        self.declare_parameter('default_bead_length', 0.100)

        cfg = str(self.get_parameter('plates_config').value).strip()
        self._plates_path = cfg if cfg else os.environ.get('ROBIN_PLATES_CONFIG', '')
        if not self._plates_path:
            self._plates_path = os.path.join(
                get_package_share_directory('robin_bringup'),
                'config',
                'plates.json',
            )

        beads_cfg = str(self.get_parameter('beads_config').value).strip()
        self._beads_path = beads_cfg if beads_cfg else os.environ.get(
            'ROBIN_BEADS_CONFIG',
            os.path.join(
                get_package_share_directory('robin_bringup'),
                'config',
                'beads.json',
            ))

        self._spacing_x = float(self.get_parameter('default_spacing_x').value)
        self._spacing_y = float(self.get_parameter('default_spacing_y').value)
        self._bead_length = float(self.get_parameter('default_bead_length').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(
            MarkerArray,
            'robin/plates/markers',
            qos,
        )
        self.get_logger().info('Configured')

        period = float(self.get_parameter('publish_period').value)
        self._timer = self.create_timer(max(0.1, period), self._publish)
        self.get_logger().info('Activated: publishing plate markers')

    @staticmethod
    def _load_plates(path: str) -> list[dict[str, Any]]:
        if not os.path.isfile(path):
            return []
        try:
            with open(path) as f:
                payload = json.load(f)
            plates = payload if isinstance(payload, list) else payload.get('plates', [])
            return plates if isinstance(plates, list) else []
        except Exception:
            return []

    def _publish(self):
        plates = self._load_plates(self._plates_path)
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        marker_id = 0

        # Clear all previous markers so stale ones don't linger
        delete_all = Marker()
        delete_all.header.frame_id = self._frame_id
        delete_all.header.stamp = now
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        for plate in plates:
            pid = str(plate.get('plate_id', f'plate_{marker_id}'))
            corner_x = float(plate.get('corner_x', plate.get('origin_x', 0.0)))
            corner_y = float(plate.get('corner_y', plate.get('origin_y', 0.0)))
            width = float(plate.get('width', 0.15))
            length = float(plate.get('length', 0.20))
            yaw = np.radians(float(plate.get('yaw_deg', 0.0)))
            corner_id = normalize_corner_id(plate.get('corner_id', 'front_left'))
            surface_z = float(plate.get('surface_z', plate.get('corner_z', 0.0)))
            mx = float(plate.get('margin_x', DEFAULT_MARGIN_M))
            my = float(plate.get('margin_y', DEFAULT_MARGIN_M))
            plane_ok = bool(plate.get('plane_calibrated', False))
            pa = float(plate.get('plane_a', 0.0))
            pb = float(plate.get('plane_b', 0.0))
            pc = float(plate.get('plane_c', surface_z))

            def local_to_world(lx: float, ly: float) -> Point:
                p = Point()
                p.x, p.y = inward_to_world(
                    corner_x, corner_y, yaw, corner_id,
                    float(lx), float(ly))
                p.z = (pa * p.x + pb * p.y + pc) if plane_ok else surface_z
                return p

            corners = [
                local_to_world(0.0, 0.0),
                local_to_world(length, 0.0),
                local_to_world(length, width),
                local_to_world(0.0, width),
            ]

            # ── Steel plate body (CUBE, 6 mm thick, brushed-steel grey) ──────
            # Build a full 3D orientation from plate edges + plane normal
            # so the cube tilts with calibrated planes (not only yaw in XY).
            c0, c1, _c2, c3 = corners
            p0, p1, p3 = _point_to_array(c0), _point_to_array(c1), _point_to_array(c3)
            x_axis = _safe_normalize(p1 - p0, np.array([1.0, 0.0, 0.0]))
            y_raw = p3 - p0
            y_raw = y_raw - np.dot(y_raw, x_axis) * x_axis
            y_axis = _safe_normalize(y_raw, np.array([0.0, 1.0, 0.0]))
            z_axis = _safe_normalize(np.cross(x_axis, y_axis), np.array([0.0, 0.0, 1.0]))
            # Ensure plate normal always points UP (away from table).
            # Corner ordering can flip the cross-product direction for
            # certain corner_id values (e.g. rear_right, front_right).
            if z_axis[2] < 0:
                z_axis = -z_axis
            y_axis = _safe_normalize(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
            rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
            q = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]
            qx, qy, qz, qw = q[0], q[1], q[2], q[3]

            center = 0.25 * (_point_to_array(c0) + _point_to_array(c1)
                             + _point_to_array(corners[2]) + _point_to_array(c3))
            cx_surface, cy_surface, cz_surface = center

            plate_body = Marker()
            plate_body.header.frame_id = self._frame_id
            plate_body.header.stamp = now
            plate_body.ns = 'plates_body'
            plate_body.id = marker_id
            marker_id += 1
            plate_body.type = Marker.CUBE
            plate_body.action = Marker.ADD
            plate_body.pose.position.x = cx_surface - z_axis[0] * (PLATE_THICKNESS * 0.5)
            plate_body.pose.position.y = cy_surface - z_axis[1] * (PLATE_THICKNESS * 0.5)
            plate_body.pose.position.z = cz_surface - z_axis[2] * (PLATE_THICKNESS * 0.5)
            plate_body.pose.orientation.x = qx
            plate_body.pose.orientation.y = qy
            plate_body.pose.orientation.z = qz
            plate_body.pose.orientation.w = qw
            plate_body.scale.x = length
            plate_body.scale.y = width
            plate_body.scale.z = PLATE_THICKNESS
            plate_body.color = _color(0.72, 0.74, 0.78, 0.95)  # brushed steel
            arr.markers.append(plate_body)

            # ── Weld-area margin (LINE_STRIP, raised 1 mm above plate top face) ─
            inset = [
                local_to_world(mx, my),
                local_to_world(length - mx, my),
                local_to_world(length - mx, width - my),
                local_to_world(mx, width - my),
                local_to_world(mx, my),
            ]
            for _p in inset:
                _p.z += PLATE_THICKNESS * 0.5 + 0.001  # sit just above top face
            inner = Marker()
            inner.header.frame_id = self._frame_id
            inner.header.stamp = now
            inner.ns = 'plates_margin'
            inner.id = marker_id
            marker_id += 1
            inner.type = Marker.LINE_STRIP
            inner.action = Marker.ADD
            inner.scale.x = 0.002
            inner.color = _color(1.0, 0.75, 0.2, 0.9)
            inner.points = inset
            arr.markers.append(inner)

            txt = Marker()
            txt.header.frame_id = self._frame_id
            txt.header.stamp = now
            txt.ns = 'plates_text'
            txt.id = marker_id
            marker_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = local_to_world(length * 0.5, width * 0.5)
            txt.pose.position.z += PLATE_THICKNESS + 0.015
            txt.scale.z = 0.02
            txt.color = _color(0.95, 0.95, 0.95, 1.0)
            txt.text = f"{pid} {'PLANE' if plane_ok else ''}"
            arr.markers.append(txt)

            probe_points = plate.get('probe_points', [])
            if isinstance(probe_points, list) and probe_points:
                pmarker = Marker()
                pmarker.header.frame_id = self._frame_id
                pmarker.header.stamp = now
                pmarker.ns = 'plates_probe_points'
                pmarker.id = marker_id
                marker_id += 1
                pmarker.type = Marker.SPHERE_LIST
                pmarker.action = Marker.ADD
                pmarker.scale.x = 0.008
                pmarker.scale.y = 0.008
                pmarker.scale.z = 0.008
                pmarker.color = _color(1.0, 0.2, 0.2, 0.95)
                pmarker.points = []
                for q in probe_points:
                    p = Point()
                    p.x = float(q.get('x', 0.0))
                    p.y = float(q.get('y', 0.0))
                    p.z = float(q.get('z', 0.0))
                    pmarker.points.append(p)
                arr.markers.append(pmarker)

            if plane_ok:
                cx, cy = inward_to_world(
                    corner_x, corner_y, yaw, corner_id,
                    0.5 * length, 0.5 * width)
                cz = pa * cx + pb * cy + pc
                n_vec = np.array([-pa, -pb, 1.0])
                n_norm = np.linalg.norm(n_vec)
                if n_norm > 1e-6:
                    n_vec /= n_norm
                nx, ny, nz = n_vec
                normal = Marker()
                normal.header.frame_id = self._frame_id
                normal.header.stamp = now
                normal.ns = 'plates_plane_normal'
                normal.id = marker_id
                marker_id += 1
                normal.type = Marker.ARROW
                normal.action = Marker.ADD
                normal.scale.x = 0.003
                normal.scale.y = 0.006
                normal.scale.z = 0.008
                normal.color = _color(0.2, 1.0, 0.2, 0.95)
                p0 = Point(); p0.x = cx; p0.y = cy; p0.z = cz
                p1 = Point(); p1.x = cx + 0.06 * nx; p1.y = cy + 0.06 * ny; p1.z = cz + 0.06 * nz
                normal.points = [p0, p1]
                arr.markers.append(normal)

            # ── Occupied slot markers (CYLINDERs on the plate surface) ───────
            # Read all bead records for this plate and use stored local
            # coordinates.  This is spacing-independent — each marker shows
            # the physical footprint that was actually recorded.
            all_records = load_completed_beads(self._beads_path)
            plate_records = [b for b in all_records if b.get('plate_id') == pid]
            reserved_ids: set[str] = set()
            for b in plate_records:
                if b.get('reserved', False):
                    reserved_ids.add(b.get('bead_id', ''))
            for b in plate_records:
                xs = b.get('local_x_start')
                xe = b.get('local_x_end')
                yc = b.get('local_y')
                if xs is None or xe is None or yc is None:
                    continue
                p_start = local_to_world(float(xs), float(yc))
                p_end = local_to_world(float(xe), float(yc))
                dx = p_end.x - p_start.x
                dy = p_end.y - p_start.y
                dz = p_end.z - p_start.z
                sl = np.sqrt(dx * dx + dy * dy + dz * dz)
                if sl < 1e-9:
                    continue
                is_reserved = b.get('bead_id', '') in reserved_ids
                sm = Marker()
                sm.header.frame_id = self._frame_id
                sm.header.stamp = now
                sm.ns = 'plates_reserved_slots' if is_reserved else 'plates_completed_slots'
                sm.id = marker_id
                marker_id += 1
                sm.type = Marker.CYLINDER
                sm.action = Marker.ADD
                sm.scale.x = SLOT_MARKER_DIAMETER
                sm.scale.y = SLOT_MARKER_DIAMETER
                sm.scale.z = sl
                # Orange for reserved, copper for completed
                sm.color = (_color(1.0, 0.45, 0.0, 0.9) if is_reserved
                            else _color(0.85, 0.55, 0.25, 0.7))
                # Position at midpoint, raised above plate surface
                sm.pose.position.x = 0.5 * (p_start.x + p_end.x)
                sm.pose.position.y = 0.5 * (p_start.y + p_end.y)
                sm.pose.position.z = 0.5 * (p_start.z + p_end.z) + PLATE_THICKNESS * 0.5 + SLOT_MARKER_HEIGHT * 0.5
                # Rotate Z→bead direction
                u = np.array([dx, dy, dz]) / sl
                z_ax = np.array([0.0, 0.0, 1.0])
                cr = np.cross(z_ax, u)
                sin_a = float(np.linalg.norm(cr))
                dot_a = float(np.dot(z_ax, u))
                if sin_a < 1e-9:
                    if dot_a > 0:
                        sm.pose.orientation.w = 1.0
                    else:
                        sm.pose.orientation.x = 1.0
                else:
                    angle = np.arctan2(sin_a, dot_a)
                    rv = (cr / sin_a) * angle
                    rq = Rotation.from_rotvec(rv).as_quat()
                    sm.pose.orientation.x = rq[0]
                    sm.pose.orientation.y = rq[1]
                    sm.pose.orientation.z = rq[2]
                    sm.pose.orientation.w = rq[3]
                arr.markers.append(sm)

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PlateMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
