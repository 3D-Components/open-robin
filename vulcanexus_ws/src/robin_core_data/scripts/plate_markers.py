#!/usr/bin/env python3
"""Publish plate visualization markers in RViz from persisted plates.json."""

import json
import math
import os
from typing import Any

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from robin_core_planner.plate_geometry import normalize_corner_id, inward_to_world


DEFAULT_MARGIN_M = 0.040
PLATE_THICKNESS = 0.006  # 6 mm — matches typical steel substrate thickness


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _vec_sub(a: Point, b: Point) -> tuple[float, float, float]:
    return (float(a.x - b.x), float(a.y - b.y), float(a.z - b.z))


def _vec_dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vec_norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(_vec_dot(v, v))


def _vec_normalize(v: tuple[float, float, float], fallback=(1.0, 0.0, 0.0)) -> tuple[float, float, float]:
    n = _vec_norm(v)
    if n < 1e-9:
        return fallback
    return (v[0] / n, v[1] / n, v[2] / n)


def _quat_from_basis(x_axis: tuple[float, float, float],
                     y_axis: tuple[float, float, float],
                     z_axis: tuple[float, float, float]) -> tuple[float, float, float, float]:
    # Rotation matrix with basis vectors as columns.
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    qn = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if qn < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / qn, qy / qn, qz / qn, qw / qn)


class PlateMarkersNode(Node):
    def __init__(self):
        super().__init__('plate_markers')
        self.declare_parameter('plates_config', '')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic', '/robin/plates/markers')
        self.declare_parameter('publish_period', 1.0)

        cfg = str(self.get_parameter('plates_config').value).strip()
        self._plates_path = cfg if cfg else os.environ.get('ROBIN_PLATES_CONFIG', '')
        if not self._plates_path:
            self._plates_path = '/workspace/ros2_packages/src/robin_core_bringup/config/plates.json'

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('topic').value),
            10,
        )
        period = float(self.get_parameter('publish_period').value)
        self.create_timer(max(0.1, period), self._publish)

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

        for plate in plates:
            pid = str(plate.get('plate_id', f'plate_{marker_id}'))
            corner_x = float(plate.get('corner_x', plate.get('origin_x', 0.0)))
            corner_y = float(plate.get('corner_y', plate.get('origin_y', 0.0)))
            width = float(plate.get('width', 0.15))
            length = float(plate.get('length', 0.20))
            yaw = math.radians(float(plate.get('yaw_deg', 0.0)))
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
            x_axis = _vec_normalize(_vec_sub(c1, c0), fallback=(1.0, 0.0, 0.0))
            y_raw = _vec_sub(c3, c0)
            y_raw = (
                y_raw[0] - _vec_dot(y_raw, x_axis) * x_axis[0],
                y_raw[1] - _vec_dot(y_raw, x_axis) * x_axis[1],
                y_raw[2] - _vec_dot(y_raw, x_axis) * x_axis[2],
            )
            y_axis = _vec_normalize(y_raw, fallback=(0.0, 1.0, 0.0))
            z_axis = _vec_normalize(_vec_cross(x_axis, y_axis), fallback=(0.0, 0.0, 1.0))
            y_axis = _vec_normalize(_vec_cross(z_axis, x_axis), fallback=(0.0, 1.0, 0.0))
            qx, qy, qz, qw = _quat_from_basis(x_axis, y_axis, z_axis)

            cx_surface = 0.25 * (c0.x + c1.x + corners[2].x + c3.x)
            cy_surface = 0.25 * (c0.y + c1.y + corners[2].y + c3.y)
            cz_surface = 0.25 * (c0.z + c1.z + corners[2].z + c3.z)

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
                nx, ny, nz = -pa, -pb, 1.0
                norm = math.sqrt(nx * nx + ny * ny + nz * nz)
                if norm > 1e-6:
                    nx, ny, nz = nx / norm, ny / norm, nz / norm
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
