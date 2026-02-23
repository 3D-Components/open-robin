"""
FAROS Visualization Server using Viser
Displays both UR robots with joint animation driven by the React app
via a WebSocket bridge on port 8082.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
import viser
import websockets
import websockets.asyncio.server
import yourdfpy


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
VISER_PORT = 8081
WS_PORT = 8082

JOINT_NAMES: list[str] = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

INITIAL_CFG: dict[str, float] = {
    'shoulder_pan_joint': 0.0,
    'shoulder_lift_joint': -1.57,
    'elbow_joint': 0.0,
    'wrist_1_joint': -1.57,
    'wrist_2_joint': 0.0,
    'wrist_3_joint': 0.0,
}

ROBOT_COLOURS: dict[str, tuple[int, int, int]] = {
    'robot_a': (70, 130, 180),  # steel-blue  – UR5
    'robot_b': (180, 100, 60),  # warm copper – UR3
}


# ---------------------------------------------------------------------------
# Looping process trajectory - time-based sinusoidal motion.
# Each segment index offsets the phase so different segments look different.
# ---------------------------------------------------------------------------


def process_loop_cfg(
    t: float, segment_index: int, robot_key: str
) -> dict[str, float]:
    """Generate a continuous looping joint config for a process sweep.

    t:          elapsed time (seconds)
    segment_index: changes the motion pattern per segment
    robot_key:  'robotA' or 'robotB' - offsets so robots look independent
    """
    # Phase offsets per robot and per segment
    robot_phase = 0.0 if robot_key == 'robotA' else math.pi / 3
    segment_phase = (segment_index % 5) * 0.8

    p = robot_phase + segment_phase

    return {
        'shoulder_pan_joint': 0.4 * math.sin(0.5 * t + p),
        'shoulder_lift_joint': -1.4 + 0.35 * math.sin(0.7 * t + p),
        'elbow_joint': 0.5 * math.sin(0.6 * t + p + 1.0),
        'wrist_1_joint': -1.4 + 0.3 * math.sin(0.8 * t + p),
        'wrist_2_joint': 0.25 * math.sin(0.9 * t + p + 0.5),
        'wrist_3_joint': 0.5 * math.sin(1.2 * t + p),
    }


# ---------------------------------------------------------------------------
# Robot handle
# ---------------------------------------------------------------------------
@dataclass
class RobotHandle:
    name: str
    urdf: yourdfpy.URDF
    prefix: str
    base_xyz: tuple[float, float, float]
    colour: tuple[int, int, int]
    # link_name → list[(mesh_handle, visual_origin_4x4)]
    mesh_nodes: dict[str, list[tuple[object, np.ndarray]]] = field(
        default_factory=dict
    )


# ---------------------------------------------------------------------------
# Load a robot into the viser scene
# ---------------------------------------------------------------------------


def load_robot(
    server: viser.ViserServer,
    urdf_path: Path,
    scene_prefix: str,
    *,
    base_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    colour: tuple[int, int, int] = (180, 180, 180),
) -> RobotHandle:
    """Parse a URDF, add visual meshes to the scene, and return a handle."""

    urdf_model = yourdfpy.URDF.load(
        str(urdf_path),
        build_collision_scene_graph=False,
        load_collision_meshes=False,
    )

    handle = RobotHandle(
        name=urdf_path.parent.name,
        urdf=urdf_model,
        prefix=scene_prefix,
        base_xyz=base_xyz,
        colour=colour,
    )

    server.scene.add_frame(
        f'{scene_prefix}/base',
        axes_length=0.05,
        axes_radius=0.001,
        position=np.array(base_xyz, dtype=np.float64),
    )

    for link_name, link in urdf_model.link_map.items():
        for vi, visual in enumerate(link.visuals):
            if visual.geometry is None or visual.geometry.mesh is None:
                continue

            mesh_path = visual.geometry.mesh.filename
            abs_mesh = (urdf_path.parent / mesh_path).resolve()
            if not abs_mesh.exists():
                print(f'  ⚠ mesh not found: {abs_mesh}')
                continue

            mesh = trimesh.load_mesh(str(abs_mesh))
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)

            vertices = np.array(mesh.vertices, dtype=np.float32)
            faces = np.array(mesh.faces, dtype=np.uint32)

            vis_origin = np.eye(4)
            if visual.origin is not None:
                vis_origin = visual.origin

            link_tf = urdf_model.get_transform(link_name, 'world')
            full_tf = link_tf @ vis_origin

            position = np.array(full_tf[:3, 3], dtype=np.float64)
            position += np.array(base_xyz, dtype=np.float64)
            wxyz = np.array(
                trimesh.transformations.quaternion_from_matrix(full_tf),
                dtype=np.float64,
            )

            node_name = f'{scene_prefix}/{link_name}/visual_{vi}'
            r, g, b = colour
            mesh_handle = server.scene.add_mesh_simple(
                node_name,
                vertices=vertices,
                faces=faces,
                color=(r, g, b),
                wireframe=False,
                position=position,
                wxyz=wxyz,
            )

            handle.mesh_nodes.setdefault(link_name, []).append(
                (mesh_handle, vis_origin)
            )

    print(f'  ✓ Loaded {urdf_path.name} → {scene_prefix}')
    return handle


# ---------------------------------------------------------------------------
# Update a robot's pose given joint angles
# ---------------------------------------------------------------------------


def update_robot_pose(
    handle: RobotHandle,
    cfg: dict[str, float],
) -> None:
    """Recompute FK and update every mesh node in the viser scene."""
    handle.urdf.update_cfg(cfg)

    for link_name, nodes in handle.mesh_nodes.items():
        link_tf = handle.urdf.get_transform(link_name, 'world')
        for mesh_handle, vis_origin in nodes:
            full_tf = link_tf @ vis_origin
            position = np.array(full_tf[:3, 3], dtype=np.float64)
            position += np.array(handle.base_xyz, dtype=np.float64)
            wxyz = np.array(
                trimesh.transformations.quaternion_from_matrix(full_tf),
                dtype=np.float64,
            )
            mesh_handle.position = position
            mesh_handle.wxyz = wxyz


# ---------------------------------------------------------------------------
# Shared state - written by WebSocket thread, read by main loop
# ---------------------------------------------------------------------------
@dataclass
class RobotState:
    state: str = 'Idle'
    segment_index: int = 0
    progress_pct: float = 0.0


latest_state: dict[str, RobotState] = {
    'robotA': RobotState(),
    'robotB': RobotState(),
}
state_lock = threading.Lock()


# ---------------------------------------------------------------------------
# WebSocket server (runs in a background thread)
# ---------------------------------------------------------------------------


async def _ws_handler(
    websocket: websockets.asyncio.server.ServerConnection,
) -> None:
    """Handle incoming robot state messages from the React app."""
    print('React app connected via WebSocket')
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            with state_lock:
                for key in ('robotA', 'robotB'):
                    if key in data:
                        rd = data[key]
                        latest_state[key].state = rd.get('state', 'Idle')
                        latest_state[key].segment_index = rd.get('segmentIndex', rd.get('beadIndex', 0))
                        latest_state[key].progress_pct = rd.get(
                            'progressPct', 0.0
                        )
    except websockets.exceptions.ConnectionClosed:
        print('React app disconnected')


async def _run_ws_server() -> None:
    async with websockets.asyncio.server.serve(
        _ws_handler,
        '0.0.0.0',
        WS_PORT,
    ):
        print(f'  WebSocket bridge listening on ws://localhost:{WS_PORT}')
        await asyncio.Future()  # run forever


def _start_ws_thread() -> None:
    """Start the WebSocket server in a daemon thread."""

    def _target() -> None:
        asyncio.run(_run_ws_server())

    t = threading.Thread(target=_target, daemon=True)
    t.start()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    server = viser.ViserServer(host='0.0.0.0', port=VISER_PORT)
    print(f'Viser server running at http://localhost:{VISER_PORT}')

    assets_root = (
        Path(__file__).resolve().parent.parent / 'frontend/public/assets'
    )

    # ── Workpiece ─────────────────────────────────────────────────────
    stl_path = (
        assets_root / 'motion/workpieces/workpiece_plate_300x200x10mm.stl'
    )
    if stl_path.exists():
        mesh = trimesh.load_mesh(str(stl_path))
        server.scene.add_mesh_simple(
            '/workpiece/plate',
            vertices=np.array(mesh.vertices, dtype=np.float32),
            faces=np.array(mesh.faces, dtype=np.uint32),
            color=(0.7, 0.7, 0.75),
            wireframe=False,
        )
        print(f'Loaded workpiece: {stl_path.name}')

    # ── Robots ────────────────────────────────────────────────────────
    robots_root = assets_root / 'robots'
    handles: dict[str, RobotHandle] = {}

    print('\nLoading Robot A (UR5) …')
    handles['robotA'] = load_robot(
        server,
        robots_root / 'robot_a/robot.urdf',
        '/robot_a',
        base_xyz=(0.0, 0.0, 0.0),
        colour=ROBOT_COLOURS['robot_a'],
    )

    print('Loading Robot B (UR3) …')
    handles['robotB'] = load_robot(
        server,
        robots_root / 'robot_b/robot.urdf',
        '/robot_b',
        base_xyz=(0.8, 0.0, 0.0),
        colour=ROBOT_COLOURS['robot_b'],
    )

    # ── Scene helpers ─────────────────────────────────────────────────
    server.scene.add_frame('/world', axes_length=0.1, axes_radius=0.002)
    server.scene.add_grid('/grid', width=1.5, height=1.5)

    # ── Start WebSocket bridge ────────────────────────────────────────
    _start_ws_thread()

    print('\nBoth robots loaded ✓')
    print(f'Viser:     http://localhost:{VISER_PORT}')
    print(f'WS bridge: ws://localhost:{WS_PORT}')
    print('Press Ctrl+C to stop')

    # ── Main animation loop ──────────────────────────────────────────
    # Per-robot animation time (only advances when Running)
    anim_time: dict[str, float] = {'robotA': 0.0, 'robotB': 0.0}
    current_cfg: dict[str, dict[str, float]] = {
        'robotA': dict(INITIAL_CFG),
        'robotB': dict(INITIAL_CFG),
    }

    dt = 0.033  # ~30 fps

    try:
        while True:
            with state_lock:
                snapshot = {
                    k: RobotState(
                        state=latest_state[k].state,
                        segment_index=latest_state[k].segment_index,
                        progress_pct=latest_state[k].progress_pct,
                    )
                    for k in ('robotA', 'robotB')
                }

            for robot_key in ('robotA', 'robotB'):
                rs = snapshot[robot_key]
                rh = handles[robot_key]

                if rs.state == 'Running':
                    # Advance animation time and compute looping joint config
                    anim_time[robot_key] += dt
                    target = process_loop_cfg(
                        anim_time[robot_key],
                        rs.segment_index,
                        robot_key,
                    )
                    update_robot_pose(rh, target)
                    current_cfg[robot_key] = target

                elif rs.state == 'Idle':
                    # Smoothly return to initial pose
                    alpha = 0.1
                    smoothed = {
                        jn: current_cfg[robot_key][jn]
                        + alpha
                        * (INITIAL_CFG[jn] - current_cfg[robot_key][jn])
                        for jn in JOINT_NAMES
                    }
                    if any(
                        abs(smoothed[jn] - INITIAL_CFG[jn]) > 0.001
                        for jn in JOINT_NAMES
                    ):
                        update_robot_pose(rh, smoothed)
                    current_cfg[robot_key] = smoothed
                    anim_time[robot_key] = 0.0

                # 'Paused' → keep current pose, don't advance time

            time.sleep(dt)
    except KeyboardInterrupt:
        print('\nServer stopped')


if __name__ == '__main__':
    main()
