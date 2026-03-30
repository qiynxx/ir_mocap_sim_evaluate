"""
Scene management: board objects, noise generator, and math helpers.

Extracted and adapted from ir_mocap_sim_evaluate/mocap_sim.py.
"""

import json
import math
import os
import time
import numpy as np


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Euler angles (degrees) -> 3x3 rotation matrix (Rz @ Ry @ Rx)."""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, math.cos(roll), -math.sin(roll)],
                   [0, math.sin(roll),  math.cos(roll)]])

    Ry = np.array([[ math.cos(pitch), 0, math.sin(pitch)],
                   [0, 1, 0],
                   [-math.sin(pitch), 0, math.cos(pitch)]])

    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw),  math.cos(yaw), 0],
                   [0, 0, 1]])

    return Rz @ Ry @ Rx


def rotation_matrix_to_euler_angles(R):
    """3x3 rotation matrix -> (roll, pitch, yaw) in degrees."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z]) * 180.0 / math.pi


# ---------------------------------------------------------------------------
# Board colours (RGB 0-255, used by the OpenGL renderer)
# ---------------------------------------------------------------------------
COLOR_BOARD1 = (100, 220, 180)
COLOR_BOARD2 = (220, 140, 255)


# ---------------------------------------------------------------------------
# BoardObject
# ---------------------------------------------------------------------------

class BoardObject:
    """LED board loaded from a JSON config entry."""

    def __init__(self, board_config, board_index):
        points_3d_mm = np.array(board_config["points_3d"], dtype=np.float32)
        self.model_points = points_3d_mm / 1000.0  # mm -> m

        self.cylinder_radius = board_config.get("cylinder_radius", 60) / 1000.0
        self.height = board_config.get("height", 40) / 1000.0
        self.width = board_config.get("width", 200) / 1000.0

        self.position = np.array([0.5, 0.0, 0.0], dtype=np.float32)
        self.rotation = np.eye(3, dtype=np.float32)
        self.euler_angles = np.array([0.0, 0.0, 0.0])

        self.name = board_config.get("name", f"Board {board_index}")
        self.color = COLOR_BOARD1 if board_index == 0 else COLOR_BOARD2

        # Optional STL for occlusion (path set in load_board_config / CLI)
        self.mesh_stl_path = None
        self.mesh_units = board_config.get("mesh_units", "mm")

        # World→local transform for aligning STL (whose vertices are in the
        # design tool's world frame) with the LED local coordinates.
        self.mesh_origin_world_mm = np.array(
            board_config.get("mesh_origin_world_mm", [0, 0, 0]),
            dtype=np.float64,
        )
        self.mesh_axis_rotation = board_config.get(
            "mesh_axis_rotation", [0, 0, 0]
        )

    def get_world_points(self):
        return np.dot(self.model_points, self.rotation.T) + self.position

    def set_pose_euler(self, pos, euler):
        self.position = pos
        self.euler_angles = euler
        self.rotation = euler_to_rotation_matrix(euler[0], euler[1], euler[2]).astype(np.float32)


# ---------------------------------------------------------------------------
# NoiseGenerator
# ---------------------------------------------------------------------------

class NoiseGenerator:
    """Generates random 3D noise points in world space."""

    def __init__(self):
        self.active = False
        self.noise_points = []
        self._last_update = 0.0
        self.update_interval = 0.1  # seconds

    def toggle(self):
        self.active = not self.active
        if not self.active:
            self.noise_points = []

    def update(self):
        if not self.active:
            return
        now = time.monotonic()
        if now - self._last_update < self.update_interval:
            return
        self._last_update = now
        self.noise_points = []

        for _ in range(5):
            self.noise_points.append(np.random.rand(3) * 4 - 2)

        center = np.random.rand(3) * 4 - 2
        for _ in range(4):
            offset = (np.random.rand(3) - 0.5) * 0.2
            self.noise_points.append(center + offset)

    def get_noise_world_points(self):
        if not self.active:
            return np.empty((0, 3))
        return np.array(self.noise_points)


# ---------------------------------------------------------------------------
# Config loader
# ---------------------------------------------------------------------------

def load_board_config(config_path):
    """Load board config JSON and return list of BoardObject."""
    with open(config_path, "r") as f:
        config = json.load(f)

    config_dir = os.path.dirname(os.path.abspath(config_path))
    boards = []
    n_boards = len(config.get("boards", []))
    for i, bc in enumerate(config.get("boards", [])):
        board = BoardObject(bc, i)

        if n_boards == 1:
            board.position = np.array([0.5, 0.0, 0.0], dtype=np.float32)
        else:
            board.position = np.array(
                [0.5, 0.3 if i == 0 else -0.3, 0.0], dtype=np.float32)

        rel = bc.get("mesh_stl")
        if rel:
            mp = rel if os.path.isabs(rel) else os.path.join(config_dir, rel)
            board.mesh_stl_path = os.path.normpath(mp)

        # Auto-orient board so LED surface faces camera (-X direction).
        init_yaw = bc.get("initial_yaw_deg")
        if init_yaw is not None:
            board.set_pose_euler(board.position,
                                np.array([0.0, 0.0, float(init_yaw)]))

        boards.append(board)
    return boards


def apply_cli_mesh_specs(boards, specs):
    """
    Apply --mesh CLI entries after JSON load. Each spec is 'IDX:path' or 'path' (board 0).
    """
    if not specs:
        return
    for spec in specs:
        spec = spec.strip()
        if not spec:
            continue
        if ":" in spec:
            idx_str, path = spec.split(":", 1)
            idx = int(idx_str.strip())
            path = path.strip()
        else:
            idx, path = 0, spec.strip()
        path = os.path.abspath(os.path.expanduser(path))
        if idx < 0 or idx >= len(boards):
            raise ValueError(f"--mesh board index {idx} out of range (0..{len(boards)-1})")
        if not os.path.isfile(path):
            print(f"[occlusion] Warning: --mesh file not found, ignoring: {path}")
            boards[idx].mesh_stl_path = None
        else:
            boards[idx].mesh_stl_path = path


def apply_points_3d_override(boards, json_path, board_index=0):
    """
    Load JSON with top-level 'points_3d' (mm) and optional 'point_ids', apply to one board.
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    pts = data.get("points_3d")
    if pts is None:
        raise ValueError(f"{json_path}: missing 'points_3d' array")
    if board_index < 0 or board_index >= len(boards):
        raise ValueError(f"points board index {board_index} out of range")
    boards[board_index].model_points = np.array(pts, dtype=np.float32) / 1000.0
