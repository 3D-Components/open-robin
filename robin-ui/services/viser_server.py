"""
ROBIN Visualization Server using Viser
Displays a UR5 robot with joint animation driven by the dashboard
via a WebSocket bridge on port 8082.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
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

# Torch-down welding posture: robot leaning over the workpiece.
WELD_BASE_CFG: dict[str, float] = {
    'shoulder_pan_joint': 0.0,
    'shoulder_lift_joint': -0.90,
    'elbow_joint': 1.80,
    'wrist_1_joint': -2.46,
    'wrist_2_joint': -1.57,
    'wrist_3_joint': 0.0,
}

# shoulder_pan sweep range for travelling along the weld seam
WELD_PAN_START = -0.35
WELD_PAN_END = 0.35

# Torch weave amplitude and frequency (small lateral oscillation)
WEAVE_AMP = 0.04
WEAVE_FREQ = 3.0  # Hz

ROBOT_COLOUR: tuple[int, int, int] = (70, 130, 180)  # steel-blue – UR5


# ---------------------------------------------------------------------------
# Welding trajectory: slow seam sweep + torch weave, driven by progress %.
# ---------------------------------------------------------------------------


def welding_cfg(
    progress_pct: float, t: float,
) -> dict[str, float]:
    """Joint config for a welding sweep along a seam.

    progress_pct:  0-100, drives position along the weld seam
    t:             elapsed time in seconds, drives the weave oscillation
    """
    frac = max(0.0, min(1.0, progress_pct / 100.0))

    pan = WELD_PAN_START + (WELD_PAN_END - WELD_PAN_START) * frac
    weave = WEAVE_AMP * math.sin(WEAVE_FREQ * 2 * math.pi * t)

    return {
        'shoulder_pan_joint': pan,
        'shoulder_lift_joint': WELD_BASE_CFG['shoulder_lift_joint'],
        'elbow_joint': WELD_BASE_CFG['elbow_joint'],
        'wrist_1_joint': WELD_BASE_CFG['wrist_1_joint'],
        'wrist_2_joint': WELD_BASE_CFG['wrist_2_joint'] + weave,
        'wrist_3_joint': WELD_BASE_CFG['wrist_3_joint'],
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


latest_state = RobotState()
state_lock = threading.Lock()


# ---------------------------------------------------------------------------
# WebSocket server (runs in a background thread)
# ---------------------------------------------------------------------------


async def _ws_handler(
    websocket: websockets.asyncio.server.ServerConnection,
) -> None:
    """Handle incoming robot state messages from the dashboard."""
    print('Dashboard connected via WebSocket')
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            rd = data.get('robotA') or data.get('robot')
            if rd:
                with state_lock:
                    latest_state.state = rd.get('state', 'Idle')
                    latest_state.segment_index = rd.get('segmentIndex', rd.get('beadIndex', 0))
                    latest_state.progress_pct = rd.get('progressPct', 0.0)
    except websockets.exceptions.ConnectionClosed:
        print('Dashboard disconnected')


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

    assets_root = Path(
        os.getenv(
            'VISER_ASSETS_ROOT',
            str(Path(__file__).resolve().parent.parent / 'frontend/public/assets'),
        )
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

    # ── Robot ─────────────────────────────────────────────────────────
    robots_root = assets_root / 'robots'

    print('\nLoading Robot A (UR5) …')
    robot_handle = load_robot(
        server,
        robots_root / 'robot_a/robot.urdf',
        '/robot_a',
        base_xyz=(0.0, 0.0, 0.0),
        colour=ROBOT_COLOUR,
    )

    # ── Scene helpers ─────────────────────────────────────────────────
    server.scene.add_frame('/world', axes_length=0.1, axes_radius=0.002)
    server.scene.add_grid('/grid', width=1.5, height=1.5)

    # ── Start WebSocket bridge ────────────────────────────────────────
    _start_ws_thread()

    print('\nRobot loaded ✓')
    print(f'Viser:     http://localhost:{VISER_PORT}')
    print(f'WS bridge: ws://localhost:{WS_PORT}')

    # ── Main animation loop ──────────────────────────────────────────
    anim_time = 0.0
    idle_target = welding_cfg(0, 0)
    current_cfg = dict(INITIAL_CFG)
    dt = 0.033  # ~30 fps

    try:
        while True:
            with state_lock:
                rs = RobotState(
                    state=latest_state.state,
                    segment_index=latest_state.segment_index,
                    progress_pct=latest_state.progress_pct,
                )

            if rs.state == 'Running':
                anim_time += dt
                target = welding_cfg(rs.progress_pct, anim_time)
                update_robot_pose(robot_handle, target)
                current_cfg = target

            elif rs.state == 'Idle':
                alpha = 0.08
                smoothed = {
                    jn: current_cfg[jn]
                    + alpha * (idle_target[jn] - current_cfg[jn])
                    for jn in JOINT_NAMES
                }
                if any(
                    abs(smoothed[jn] - idle_target[jn]) > 0.001
                    for jn in JOINT_NAMES
                ):
                    update_robot_pose(robot_handle, smoothed)
                current_cfg = smoothed
                anim_time = 0.0

            # 'Paused' → keep current pose, don't advance time

            time.sleep(dt)
    except KeyboardInterrupt:
        print('\nServer stopped')


if __name__ == '__main__':
    main()
