"""
Mocap Simulation Publisher

Interactive 3D scene with virtual stereo IR cameras.  Generates IR grayscale
images and publishes them over ZMQ in the format expected by mocap_ir_cpp:

    [uint64_t timestamp_ns  (8 bytes, little-endian)][JPEG data]

Each camera publishes on its own ZMQ PUB socket (default ports 5552 / 5553).

Usage:
    python sim_publisher.py
    python sim_publisher.py -c config/mocap_config.json --calib config/calibration_sim.json
    python sim_publisher.py --mesh 0:config/meshes/board.stl
    python sim_publisher.py --mesh path/to/model.stl --points-3d path/to/leds.json
    python sim_publisher.py --port 5552 --fps 30
    python sim_publisher.py --trajectory config/trajectories/sweep.json
"""

import argparse
import csv
import json
import math
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np
import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import zmq

from sim_camera import SimStereoCamera
from sim_evaluator import PoseEvaluator
from sim_ir_renderer import IRImageGenerator
from sim_occlusion import OcclusionEngine
from sim_scene import (
    NoiseGenerator, load_board_config,
    apply_cli_mesh_specs, apply_points_3d_override,
    euler_to_rotation_matrix, rotation_matrix_to_euler_angles,
    COLOR_BOARD1, COLOR_BOARD2,
)


# ---------------------------------------------------------------------------
# Preset poses: typical test scenarios
# ---------------------------------------------------------------------------
PRESET_POSES = [
    {
        "name": "Front Close",
        "description": "0.4m, facing camera",
        "position": [0.4, 0.0, 0.0],
        "euler": [0.0, 0.0, 0.0],
    },
    {
        "name": "Front Mid",
        "description": "1.0m, facing camera",
        "position": [1.0, 0.0, 0.0],
        "euler": [0.0, 0.0, 0.0],
    },
    {
        "name": "Front Far",
        "description": "2.5m, facing camera",
        "position": [2.5, 0.0, 0.0],
        "euler": [0.0, 0.0, 0.0],
    },
    {
        "name": "Left 30\u00b0",
        "description": "1.0m, offset left, rotated 30\u00b0",
        "position": [0.9, 0.4, 0.0],
        "euler": [0.0, 0.0, -30.0],
    },
    {
        "name": "Right 30\u00b0",
        "description": "1.0m, offset right, rotated -30\u00b0",
        "position": [0.9, -0.4, 0.0],
        "euler": [0.0, 0.0, 30.0],
    },
    {
        "name": "Above 45\u00b0",
        "description": "0.8m, elevated, pitched 45\u00b0",
        "position": [0.6, 0.0, 0.4],
        "euler": [0.0, -45.0, 0.0],
    },
    {
        "name": "Oblique",
        "description": "1.2m, combined offset and rotation",
        "position": [1.0, 0.3, 0.2],
        "euler": [15.0, -20.0, 25.0],
    },
    {
        "name": "Extreme Angle",
        "description": "0.6m, steep yaw 60\u00b0",
        "position": [0.5, 0.5, 0.0],
        "euler": [0.0, 0.0, -60.0],
    },
    {
        "name": "Far Tilted",
        "description": "3.0m, slight tilt",
        "position": [3.0, 0.0, 0.1],
        "euler": [10.0, 5.0, -5.0],
    },
]


# ---------------------------------------------------------------------------
# Trajectory player
# ---------------------------------------------------------------------------
class TrajectoryPlayer:
    """Plays back a sequence of waypoints with smooth interpolation.

    Trajectory JSON format::

        {
            "name": "sweep",
            "loop": true,
            "dwell_frames": 30,
            "waypoints": [
                {"position": [x,y,z], "euler": [r,p,y]},
                ...
            ]
        }

    ``dwell_frames`` is how many simulation frames to hold each waypoint
    before interpolating to the next (default 30 = ~1 second at 30 FPS).
    Set ``"interpolate": false`` to snap without smooth transitions.
    """

    def __init__(self):
        self.active = False
        self.waypoints = []
        self.loop = True
        self.dwell_frames = 30
        self.interpolate = True
        self.name = ""

        self._wp_index = 0
        self._frame_in_segment = 0
        self._transition_frames = 15  # frames to interpolate between waypoints
        self._done = False

        # Error logging
        self._log_path = None
        self._log_file = None
        self._log_writer = None

    def load(self, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.waypoints = data.get("waypoints", [])
        self.loop = data.get("loop", True)
        self.dwell_frames = data.get("dwell_frames", 30)
        self.interpolate = data.get("interpolate", True)
        self.name = data.get("name", os.path.basename(path))
        self._wp_index = 0
        self._frame_in_segment = 0
        self._done = False
        if not self.waypoints:
            print(f"[trajectory] WARNING: no waypoints in {path}")
        else:
            print(f"[trajectory] loaded '{self.name}': {len(self.waypoints)} waypoints, "
                  f"loop={self.loop}, dwell={self.dwell_frames}")

    def load_from_presets(self):
        """Build a trajectory from all built-in presets."""
        self.waypoints = [
            {"position": p["position"], "euler": p["euler"]}
            for p in PRESET_POSES
        ]
        self.loop = True
        self.dwell_frames = 60
        self.interpolate = True
        self.name = "All Presets"
        self._wp_index = 0
        self._frame_in_segment = 0
        self._done = False
        print(f"[trajectory] loaded preset sweep: {len(self.waypoints)} waypoints")

    def toggle(self):
        if not self.waypoints:
            self.load_from_presets()
        self.active = not self.active
        if self.active:
            self._done = False
            print(f"[trajectory] playback STARTED: '{self.name}'")
        else:
            print("[trajectory] playback PAUSED")

    def start_logging(self, log_dir):
        """Start CSV error logging for the current trajectory run."""
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(log_dir, f"traj_{self.name}_{ts}.csv")
        self._log_file = open(self._log_path, "w", newline="", encoding="utf-8")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            "frame", "waypoint", "board_name",
            "gt_x", "gt_y", "gt_z", "gt_r", "gt_p", "gt_yaw",
            "est_x", "est_y", "est_z",
            "pos_err_mm", "rot_err_deg", "rmse_mm", "inliers",
        ])
        print(f"[trajectory] logging errors to {self._log_path}")

    def stop_logging(self):
        if self._log_file:
            self._log_file.close()
            print(f"[trajectory] log saved: {self._log_path}")
            self._log_file = None
            self._log_writer = None

    def log_frame(self, frame_id, evaluator, boards):
        if self._log_writer is None:
            return
        for board in boards:
            err = evaluator.errors.get(board.name)
            if err is None:
                continue
            p = board.position
            e = board.euler_angles
            ep = err.estimated_pos
            self._log_writer.writerow([
                frame_id, self._wp_index, board.name,
                f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}",
                f"{e[0]:.2f}", f"{e[1]:.2f}", f"{e[2]:.2f}",
                f"{ep[0]:.6f}", f"{ep[1]:.6f}", f"{ep[2]:.6f}",
                f"{err.pos_error_mm:.3f}", f"{err.rot_error_deg:.3f}",
                f"{err.rmse_mm:.3f}", err.num_inliers,
            ])

    def step(self, board):
        """Advance one frame. Returns (position, euler) for the board."""
        if not self.active or not self.waypoints or self._done:
            return None

        total_segment = self.dwell_frames + (self._transition_frames if self.interpolate else 0)
        wp = self.waypoints[self._wp_index]
        pos_target = np.array(wp["position"], dtype=np.float32)
        euler_target = np.array(wp["euler"], dtype=np.float64)

        if self.interpolate and self._frame_in_segment < self._transition_frames:
            prev_idx = (self._wp_index - 1) % len(self.waypoints)
            if self._wp_index == 0 and not self.loop:
                pos_result = pos_target
                euler_result = euler_target
            else:
                prev = self.waypoints[prev_idx]
                t = self._frame_in_segment / max(1, self._transition_frames)
                t = t * t * (3 - 2 * t)  # smoothstep
                pos_prev = np.array(prev["position"], dtype=np.float32)
                euler_prev = np.array(prev["euler"], dtype=np.float64)
                pos_result = pos_prev + (pos_target - pos_prev) * t
                euler_result = euler_prev + (euler_target - euler_prev) * t
        else:
            pos_result = pos_target
            euler_result = euler_target

        self._frame_in_segment += 1
        if self._frame_in_segment >= total_segment:
            self._frame_in_segment = 0
            self._wp_index += 1
            if self._wp_index >= len(self.waypoints):
                if self.loop:
                    self._wp_index = 0
                else:
                    self._done = True
                    self.active = False
                    print("[trajectory] playback FINISHED")

        return pos_result, euler_result

    @property
    def status_text(self):
        if not self.waypoints:
            return "No trajectory"
        if self._done:
            return f"Done ({self.name})"
        if self.active:
            return f"Playing {self._wp_index+1}/{len(self.waypoints)}"
        return f"Paused {self._wp_index+1}/{len(self.waypoints)}"


# ---------------------------------------------------------------------------
# UI Theme (refined dark)
# ---------------------------------------------------------------------------
class Theme:
    BG        = (16, 18, 27)
    PANEL     = (24, 27, 40)
    BORDER    = (48, 54, 76)
    TEXT      = (225, 232, 245)
    TEXT_DIM  = (125, 138, 165)
    ACCENT    = (105, 115, 255)
    SUCCESS   = (52, 211, 153)
    WARNING   = (250, 190, 40)
    ERROR     = (248, 113, 113)
    INFO      = (96, 165, 250)
    LED       = (255, 100, 100)
    RECON     = (103, 232, 249)
    NOISE_CLR = (251, 146, 60)
    SELECTED  = (250, 204, 21)
    GRID_MAJ  = (45, 50, 68)
    GRID_MIN  = (30, 34, 48)
    TITLE_BG  = (30, 34, 52)
    TITLE_TXT = (225, 232, 245)
    SECTION   = (96, 165, 250)
    PANEL_ACC = (32, 38, 58)
    IR_BORDER = (70, 140, 220)


# ---------------------------------------------------------------------------
# Lightweight text renderer (Pygame font -> GL texture, with cache)
# ---------------------------------------------------------------------------
class TextRenderer:
    def __init__(self):
        self._fonts = {}
        self._cache = {}
        for name, size in [("small", 12), ("normal", 14), ("medium", 16)]:
            self._fonts[name] = pygame.font.SysFont("Arial", size)

    def draw(self, text, x, y, color=Theme.TEXT, font="normal"):
        key = (text, font, color)
        if key not in self._cache:
            surf = self._fonts[font].render(text, True, color)
            data = pygame.image.tostring(surf, "RGBA", True)
            w, h = surf.get_size()
            tex = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, tex)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, data)
            self._cache[key] = (tex, w, h)

        tex, w, h = self._cache[key]
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBindTexture(GL_TEXTURE_2D, tex)
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 1); glVertex2f(x, y)
        glTexCoord2f(1, 1); glVertex2f(x + w, y)
        glTexCoord2f(1, 0); glVertex2f(x + w, y + h)
        glTexCoord2f(0, 0); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_BLEND)

    def clear(self):
        for tex, _, _ in self._cache.values():
            try:
                glDeleteTextures([tex])
            except Exception:
                pass
        self._cache.clear()


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class SimPublisher:
    def __init__(self, args):
        # --- Pygame / OpenGL init ---
        pygame.init()
        pygame.font.init()
        info = pygame.display.Info()
        self.win_w = info.current_w // 2
        self.win_h = info.current_h // 2
        pygame.display.set_caption("Mocap Sim Publisher")
        self.screen = pygame.display.set_mode(
            (self.win_w, self.win_h), DOUBLEBUF | OPENGL | RESIZABLE
        )
        self.clock = pygame.time.Clock()
        pygame.key.set_repeat(1, 16)

        self._init_gl()
        self.text = TextRenderer()

        # --- Scene ---
        self.boards = load_board_config(args.config)
        if args.points_3d_json:
            apply_points_3d_override(self.boards, args.points_3d_json, args.points_board)
        apply_cli_mesh_specs(self.boards, args.mesh or [])

        self.noise_gen = NoiseGenerator()
        self.selected_board = 0

        self.occlusion = OcclusionEngine(
            self.boards,
            enabled=not args.no_occlusion,
            eps_m=args.occlusion_eps_mm / 1000.0,
            backface_cull=not args.no_backface_cull,
        )
        self._vis_stats = [(0, 0, 0) for _ in self.boards]  # (vis_L, vis_R, total)
        self._last_vis_L = None
        self._last_vis_R = None
        self._occlusion_dirty = True

        # Cache mesh geometry for OpenGL rendering (scaled to metres).
        # Display lists are compiled lazily after the GL context is ready.
        self._board_mesh_data = []
        self._mesh_display_lists = []
        for bi, board in enumerate(self.boards):
            md = self.occlusion.get_mesh_data(bi)
            self._board_mesh_data.append(md)
            self._mesh_display_lists.append(None)  # compiled on first draw

        # --- Camera ---
        self.cam = SimStereoCamera(args.calib)

        # --- IR generators (one per eye) ---
        self.ir_gen_L = IRImageGenerator(self.cam.width, self.cam.height)
        self.ir_gen_R = IRImageGenerator(self.cam.width, self.cam.height)
        self.ir_image_L = None
        self.ir_image_R = None
        # Per-camera LED projections for UI overlays (LEDs only, no noise):
        # visible = not occluded; occluded = geometrically blocked in that eye.
        self._led_px_L = []       # visible LEDs in left IR preview
        self._led_px_R = []       # visible LEDs in right IR preview
        self._led_px_L_occ = []   # occluded LEDs in left IR preview
        self._led_px_R_occ = []   # occluded LEDs in right IR preview

        # --- ZMQ publish ---
        self.zmq_ctx = zmq.Context()
        self.pub_left = self.zmq_ctx.socket(zmq.PUB)
        self.pub_right = self.zmq_ctx.socket(zmq.PUB)
        addr_left = f"tcp://*:{args.port}"
        addr_right = f"tcp://*:{args.port + 1}"
        self.pub_left.bind(addr_left)
        self.pub_right.bind(addr_right)
        print(f"ZMQ PUB bound  left={addr_left}  right={addr_right}")

        self.jpeg_quality = args.jpeg_quality
        self.target_fps = args.fps

        # --- God-view orbit camera ---
        self.cam_rot = [30.0, 0.0]
        self.cam_zoom = -3.0
        self._drag_left = False   # left button: orbit view
        self._drag_right = False  # right button: move/rotate board
        self._toggle_held = set()

        # --- Mouse sensitivity for board control ---
        self._mouse_move_speed = 0.002   # metres per pixel
        self._mouse_rot_speed = 0.4      # degrees per pixel

        # --- ZMQ send status ---
        self.frames_sent = 0
        self._publish_fps = 0.0          # actual measured publish FPS
        self._publish_fps_ts = 0.0       # timestamp of last FPS measurement
        self._publish_fps_count = 0      # frame counter for FPS window

        # --- Preset & trajectory ---
        self._preset_index = -1
        self.trajectory = TrajectoryPlayer()
        if args.trajectory:
            self.trajectory.load(args.trajectory)

        self._log_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "logs"
        )

        # --- Evaluator (subscribe to C++ pipeline output) ---
        eval_addr = f"tcp://localhost:{args.eval_port}"
        self.evaluator = PoseEvaluator(self.boards, address=eval_addr)
        self.evaluator.start()
        self.show_eval = True
        print(f"Eval SUB connecting to {eval_addr}")

        # Report occlusion status
        if args.no_occlusion:
            print("Occlusion: OFF (--no-occlusion)")
        elif not self.occlusion.has_any_mesh():
            print("Occlusion: no mesh (use JSON mesh_stl or --mesh PATH)")
        else:
            print("Occlusion: ON (CPU STL ray cast + back-face cull)")

        self._ir_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._running = False

    # ------------------------------------------------------------------
    @staticmethod
    def _init_gl():
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # ------------------------------------------------------------------
    # Input handling
    # ------------------------------------------------------------------
    @staticmethod
    def _get_modifiers():
        """Query Shift/Ctrl via get_pressed() – reliable on Linux+OpenGL."""
        keys = pygame.key.get_pressed()
        shift = keys[K_LSHIFT] or keys[K_RSHIFT]
        ctrl = keys[K_LCTRL] or keys[K_RCTRL]
        return shift, ctrl

    def _handle_input(self):
        move_speed = 0.008
        rot_speed = 1.5

        for ev in pygame.event.get():
            if ev.type == QUIT:
                return False

            elif ev.type == VIDEORESIZE:
                self.win_w, self.win_h = ev.w, ev.h
                self.screen = pygame.display.set_mode(
                    (self.win_w, self.win_h), DOUBLEBUF | OPENGL | RESIZABLE
                )
                self._init_gl()

            # --- Mouse buttons ---
            # Left-drag: move/rotate board  |  Right-drag: orbit view
            elif ev.type == MOUSEBUTTONDOWN:
                shift, ctrl = self._get_modifiers()
                if ev.button == 1:
                    self._drag_left = True
                elif ev.button == 3:
                    self._drag_right = True
                elif ev.button == 4:  # scroll up
                    if ctrl and len(self.boards) > 0:
                        board = self.boards[self.selected_board]
                        board.position[2] += 0.02
                        board.set_pose_euler(board.position, board.euler_angles)
                        with self._pose_lock:
                            self._occlusion_dirty = True
                    else:
                        self.cam_zoom += 0.2
                elif ev.button == 5:  # scroll down
                    if ctrl and len(self.boards) > 0:
                        board = self.boards[self.selected_board]
                        board.position[2] -= 0.02
                        board.set_pose_euler(board.position, board.euler_angles)
                        with self._pose_lock:
                            self._occlusion_dirty = True
                    else:
                        self.cam_zoom -= 0.2

            elif ev.type == MOUSEBUTTONUP:
                if ev.button == 1:
                    self._drag_left = False
                elif ev.button == 3:
                    self._drag_right = False

            # --- Mouse motion ---
            elif ev.type == MOUSEMOTION:
                dx, dy = ev.rel
                if self._drag_right and not self._drag_left:
                    self.cam_rot[1] += dx * 0.5
                    self.cam_rot[0] += dy * 0.5
                elif self._drag_left and len(self.boards) > 0:
                    shift, ctrl = self._get_modifiers()
                    board = self.boards[self.selected_board]
                    if shift:
                        board.euler_angles[1] -= dy * self._mouse_rot_speed
                        board.euler_angles[2] -= dx * self._mouse_rot_speed
                    elif ctrl:
                        board.euler_angles[0] -= dx * self._mouse_rot_speed
                    else:
                        board.position[0] += dy * self._mouse_move_speed
                        board.position[1] += dx * self._mouse_move_speed
                    board.set_pose_euler(board.position, board.euler_angles)
                    with self._pose_lock:
                        self._occlusion_dirty = True

            # --- Key release (for toggle debounce) ---
            elif ev.type == KEYUP:
                self._toggle_held.discard(ev.key)

            # --- Key press ---
            elif ev.type == KEYDOWN:
                k = ev.key

                # Toggle keys
                if k == K_v and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.show_eval = not self.show_eval
                elif k == K_n and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.noise_gen.toggle()

                # Board selection
                elif k == K_TAB and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.selected_board = (self.selected_board + 1) % max(1, len(self.boards))
                elif k == K_1 and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.selected_board = 0
                elif k == K_2 and len(self.boards) > 1 and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.selected_board = 1

                # Preset poses (F1-F9)
                elif K_F1 <= k <= K_F9 and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    idx = k - K_F1
                    if idx < len(PRESET_POSES):
                        self._apply_preset(idx)

                # Trajectory playback
                elif k == K_p and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.trajectory.toggle()
                    if self.trajectory.active:
                        self.trajectory.start_logging(self._log_dir)
                    else:
                        self.trajectory.stop_logging()

                # Keyboard movement (legacy, still works as fine control)
                elif len(self.boards) > 0:
                    board = self.boards[self.selected_board]
                    moved = True
                    if   k == K_w: board.position[0] += move_speed
                    elif k == K_s: board.position[0] -= move_speed
                    elif k == K_a: board.position[1] += move_speed
                    elif k == K_d: board.position[1] -= move_speed
                    elif k == K_q: board.position[2] += move_speed
                    elif k == K_e: board.position[2] -= move_speed
                    elif k == K_r: board.euler_angles[0] += rot_speed
                    elif k == K_f: board.euler_angles[0] -= rot_speed
                    elif k == K_t: board.euler_angles[1] += rot_speed
                    elif k == K_g: board.euler_angles[1] -= rot_speed
                    elif k == K_y: board.euler_angles[2] += rot_speed
                    elif k == K_h: board.euler_angles[2] -= rot_speed
                    else:
                        moved = False
                    if moved:
                        board.set_pose_euler(board.position, board.euler_angles)
                        with self._pose_lock:
                            self._occlusion_dirty = True
        return True

    def _apply_preset(self, index):
        """Jump the selected board to a preset pose."""
        if index < 0 or index >= len(PRESET_POSES):
            return
        preset = PRESET_POSES[index]
        board = self.boards[self.selected_board]
        board.position = np.array(preset["position"], dtype=np.float32)
        board.euler_angles = np.array(preset["euler"], dtype=np.float64)
        board.set_pose_euler(board.position, board.euler_angles)
        self._preset_index = index
        print(f"[preset] F{index+1}: {preset['name']} - {preset['description']}")
        with self._pose_lock:
            self._occlusion_dirty = True

    # ------------------------------------------------------------------
    # Vision + ZMQ publish (background thread)
    # ------------------------------------------------------------------
    def _publish_loop(self):
        interval = 1.0 / max(1, self.target_fps)
        _vis_L = _vis_R = None
        next_frame = time.monotonic()
        fps_window_start = time.monotonic()
        fps_window_count = 0
        while self._running:
            now = time.monotonic()
            # Sleep most of the remaining time, then busy-wait the last 1ms
            # to compensate for OS scheduler jitter at 60 Hz.
            sleep_time = next_frame - now - 0.001
            if sleep_time > 0:
                time.sleep(sleep_time)
            while time.monotonic() < next_frame:
                pass
            # Clamp: if we're more than one interval late, skip ahead to avoid
            # a burst of catch-up frames that destabilise the cadence.
            now = time.monotonic()
            if now - next_frame > interval:
                next_frame = now
            next_frame += interval
            t0 = time.monotonic()

            with self._pose_lock:
                dirty = self._occlusion_dirty
                self._occlusion_dirty = False

            self.noise_gen.update()
            noise_pts = self.noise_gen.get_noise_world_points()

            # World-space LED positions (concatenated across all boards)
            led_pts_world = []
            for board in self.boards:
                led_pts_world.append(board.get_world_points())
            led_pts_world = (
                np.vstack(led_pts_world) if led_pts_world else np.empty((0, 3))
            )

            # ------------------------------------------------------------------
            # LED visibility masks (CPU raycast)
            # ------------------------------------------------------------------
            if dirty or _vis_L is None or _vis_R is None:
                _vis_L, _vis_R = self.occlusion.visibility_for_board_leds(
                    self.boards, self.cam.pos_left, self.cam.pos_right
                )
            vis_L, vis_R = _vis_L, _vis_R

            # Per-board visibility statistics (for left-side panel)
            o = 0
            for bi, board in enumerate(self.boards):
                n = len(board.get_world_points())
                self._vis_stats[bi] = (
                    int(vis_L[o:o + n].sum()),
                    int(vis_R[o:o + n].sum()),
                    n,
                )
                o += n

            # Cache the latest vis masks for the 3D god-view LED colouring
            self._last_vis_L = vis_L
            self._last_vis_R = vis_R

            # Build combined point cloud for IR generation (LEDs + optional noise)
            all_world = led_pts_world
            if len(noise_pts):
                all_world = np.vstack((all_world, noise_pts)) if len(all_world) else noise_pts

            noise_n = len(noise_pts)
            vis_L_ext = np.concatenate([vis_L, np.ones(noise_n, dtype=bool)]) if noise_n else vis_L
            vis_R_ext = np.concatenate([vis_R, np.ones(noise_n, dtype=bool)]) if noise_n else vis_R

            led_L_all = self.cam.project_with_distance(all_world, is_left=True)
            led_R_all = self.cam.project_with_distance(all_world, is_left=False)

            led_L = [p if (p is not None and v) else None for p, v in zip(led_L_all, vis_L_ext)]
            led_R = [p if (p is not None and v) else None for p, v in zip(led_R_all, vis_R_ext)]

            valid_L = [p for p in led_L if p is not None]
            valid_R = [p for p in led_R if p is not None]

            # ------------------------------------------------------------------
            # Overlay data for UI: split LEDs into visible / occluded per eye.
            # Only LEDs (no noise) contribute to the overlays.
            # ------------------------------------------------------------------
            n_led = len(vis_L)
            overlay_vis_L = []
            overlay_occ_L = []
            overlay_vis_R = []
            overlay_occ_R = []
            for idx in range(n_led):
                pL = led_L_all[idx]
                if pL is not None:
                    if vis_L[idx]:
                        overlay_vis_L.append(pL)
                    else:
                        overlay_occ_L.append(pL)
                pR = led_R_all[idx]
                if pR is not None:
                    if vis_R[idx]:
                        overlay_vis_R.append(pR)
                    else:
                        overlay_occ_R.append(pR)

            noise_on = self.noise_gen.active
            img_L = self.ir_gen_L.generate_image(valid_L, noise_on)
            img_R = self.ir_gen_R.generate_image(valid_R, noise_on)

            ts_ns = int(time.time() * 1e9)
            header = struct.pack("<Q", ts_ns)
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

            ok_l, jpg_l = cv2.imencode(".jpg", img_L, encode_params)
            ok_r, jpg_r = cv2.imencode(".jpg", img_R, encode_params)

            if ok_l:
                self.pub_left.send(header + jpg_l.tobytes(), zmq.NOBLOCK)
            if ok_r:
                self.pub_right.send(header + jpg_r.tobytes(), zmq.NOBLOCK)

            with self._ir_lock:
                self.ir_image_L = img_L
                self.ir_image_R = img_R
                # Overlays use LED-only projections, split into visible/occluded.
                self._led_px_L = list(overlay_vis_L)
                self._led_px_R = list(overlay_vis_R)
                self._led_px_L_occ = list(overlay_occ_L)
                self._led_px_R_occ = list(overlay_occ_R)

            self.frames_sent += 1
            fps_window_count += 1
            elapsed = time.monotonic() - fps_window_start
            if elapsed >= 0.5:
                self._publish_fps = fps_window_count / elapsed
                fps_window_start = time.monotonic()
                fps_window_count = 0

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def _draw_god_view(self):
        glViewport(0, 0, self.win_w, self.win_h)
        glClearColor(*[c / 255.0 for c in Theme.BG], 1.0)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.win_w / max(1, self.win_h), 0.1, 50.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, self.cam_zoom)
        glRotatef(self.cam_rot[0], 1, 0, 0)
        glRotatef(self.cam_rot[1], 0, 1, 0)
        # FLU -> OpenGL
        glRotatef(90, 0, 0, 1)
        glRotatef(90, 0, 1, 0)

        # Grid
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBegin(GL_LINES)
        glColor4f(*[c / 255.0 for c in Theme.GRID_MIN], 0.6)
        for i in range(-10, 11):
            if i % 5 != 0:
                glVertex3f(-2, i * 0.2, -0.5); glVertex3f(2, i * 0.2, -0.5)
                glVertex3f(i * 0.2, -2, -0.5); glVertex3f(i * 0.2, 2, -0.5)
        glEnd()
        glBegin(GL_LINES)
        glColor4f(*[c / 255.0 for c in Theme.GRID_MAJ], 0.85)
        for i in range(-2, 3):
            glVertex3f(-2, i, -0.5); glVertex3f(2, i, -0.5)
            glVertex3f(i, -2, -0.5); glVertex3f(i, 2, -0.5)
        glEnd()

        # Axes (world frame, always bright and unaffected by lighting)
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        glBegin(GL_LINES)
        # X axis: red
        glColor4f(1.0, 0.2, 0.2, 1.0)
        glVertex3f(0, 0, 0); glVertex3f(0.7, 0, 0)
        # Y axis: green / cyan
        glColor4f(0.2, 1.0, 0.5, 1.0)
        glVertex3f(0, 0, 0); glVertex3f(0, 0.7, 0)
        # Z axis: blue
        glColor4f(0.3, 0.6, 1.0, 1.0)
        glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.7)
        glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)
        glDisable(GL_BLEND)

        # Cameras (small frustums with L/R labels)
        for pos, label, clr in [
            (self.cam.pos_left,  "L", (0.2, 0.8, 1.0)),   # left: bright blue-cyan
            (self.cam.pos_right, "R", (1.0, 0.7, 0.3)),   # right: bright orange
        ]:
            glDisable(GL_LIGHTING)
            glColor3f(*clr)
            glPushMatrix()
            glTranslatef(*pos)
            self._draw_camera_body()
            glPopMatrix()
            glEnable(GL_LIGHTING)

        # Boards (mesh or wireframe fallback)
        for bi, board in enumerate(self.boards):
            pts = board.get_world_points()
            md = self._board_mesh_data[bi] if bi < len(self._board_mesh_data) else None

            if md is not None:
                self._draw_board_mesh_fast(bi, board, md)
            else:
                glColor3f(*[c / 255.0 for c in board.color])
                self._draw_connected_shape(pts)

        # LED points (depth test disabled to avoid z-fighting with mesh surface)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        vis_L_all = getattr(self, '_last_vis_L', None)
        vis_R_all = getattr(self, '_last_vis_R', None)
        led_offset = 0
        for bi, board in enumerate(self.boards):
            pts = board.get_world_points()
            n = len(pts)
            for pi, p in enumerate(pts):
                idx = led_offset + pi
                is_vis = True
                if vis_L_all is not None and idx < len(vis_L_all):
                    is_vis = bool(vis_L_all[idx]) or bool(vis_R_all[idx])
                if is_vis:
                    glColor3f(1.0, 0.3, 0.3)
                    glPointSize(8)
                else:
                    glColor3f(0.4, 0.4, 0.4)
                    glPointSize(4)
                glBegin(GL_POINTS)
                glVertex3f(*p)
                glEnd()
            led_offset += n
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        # Noise points
        if self.noise_gen.active:
            glColor3f(*[c / 255.0 for c in Theme.NOISE_CLR])
            glPointSize(4)
            glBegin(GL_POINTS)
            for p in self.noise_gen.get_noise_world_points():
                glVertex3f(*p)
            glEnd()

    def _compile_mesh_display_list(self, mesh_data, color):
        """Compile an OpenGL display list for a mesh in LOCAL coordinates."""
        verts = mesh_data["vertices"]
        faces = mesh_data["faces"]
        normals = mesh_data["face_normals"]

        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)

        # --- Pass 1: semi-transparent solid ---
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LIGHTING)
        glColor4f(color[0], color[1], color[2], 0.35)
        glBegin(GL_TRIANGLES)
        for fi in range(len(faces)):
            glNormal3fv(normals[fi])
            for vi in faces[fi]:
                glVertex3fv(verts[vi])
        glEnd()

        # --- Pass 2: sparse wireframe overlay ---
        glDisable(GL_LIGHTING)
        glColor4f(color[0], color[1], color[2], 0.5)
        glLineWidth(1)
        step = max(1, len(faces) // 2000)
        glBegin(GL_LINES)
        for fi in range(0, len(faces), step):
            f = faces[fi]
            for a, b in ((0, 1), (1, 2), (2, 0)):
                glVertex3fv(verts[f[a]])
                glVertex3fv(verts[f[b]])
        glEnd()

        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)

        glEndList()
        return dl

    def _draw_board_mesh_fast(self, board_index, board, mesh_data):
        """Render via a cached display list + GPU model-view transform."""
        if self._mesh_display_lists[board_index] is None:
            color = [c / 255.0 for c in board.color]
            self._mesh_display_lists[board_index] = \
                self._compile_mesh_display_list(mesh_data, color)
            print(f"[render] compiled display list for board {board_index} "
                  f"({len(mesh_data['faces'])} faces)")

        R = board.rotation.astype(np.float64)
        pos = board.position.astype(np.float64)

        # Build a column-major 4x4 model matrix for OpenGL
        m = np.eye(4, dtype=np.float64)
        m[:3, :3] = R
        m[:3, 3] = pos
        gl_matrix = m.T.flatten().astype(np.float32)

        glPushMatrix()
        glMultMatrixf(gl_matrix)
        glCallList(self._mesh_display_lists[board_index])
        glPopMatrix()

    @staticmethod
    def _draw_camera_body():
        s = 0.03
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0); glVertex3f(s, -s, -s)
        glVertex3f(0, 0, 0); glVertex3f(s,  s, -s)
        glVertex3f(0, 0, 0); glVertex3f(s,  s,  s)
        glVertex3f(0, 0, 0); glVertex3f(s, -s,  s)
        glVertex3f(s, -s, -s); glVertex3f(s,  s, -s)
        glVertex3f(s,  s, -s); glVertex3f(s,  s,  s)
        glVertex3f(s,  s,  s); glVertex3f(s, -s,  s)
        glVertex3f(s, -s,  s); glVertex3f(s, -s, -s)
        glEnd()
        glLineWidth(1.0)

    @staticmethod
    def _draw_connected_shape(points):
        glBegin(GL_LINE_LOOP)
        for p in points:
            glVertex3f(*p)
        glEnd()
        center = np.mean(points, axis=0)
        glBegin(GL_LINES)
        for p in points:
            glVertex3fv(center)
            glVertex3fv(p)
        glEnd()

    # ------------------------------------------------------------------
    def _draw_ir_overlay(
        self,
        ir_img,
        x,
        y,
        w,
        h,
        label,
        led_visible=None,
        led_occluded=None,
    ):
        """Draw a small IR image preview with LED markers.

        ``led_visible`` and ``led_occluded`` are lists of (u, v, d) tuples in
        full-resolution pixel coordinates (before downscaling to the preview).
        """
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.win_w, self.win_h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Shadow (drawn first, behind everything)
        glColor4f(0.0, 0.0, 0.0, 0.35)
        glBegin(GL_QUADS)
        glVertex2f(x + 3, y + 3); glVertex2f(x + w + 3, y + 3)
        glVertex2f(x + w + 3, y + h + 3); glVertex2f(x + 3, y + h + 3)
        glEnd()

        # Dark background behind image area
        glColor4f(0.04, 0.04, 0.06, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()

        # Image texture
        if ir_img is not None:
            disp = cv2.resize(ir_img, (w, h), interpolation=cv2.INTER_LINEAR)
            rgba = np.empty((h, w, 4), dtype=np.uint8)
            rgba[:, :, 0] = disp
            rgba[:, :, 1] = disp
            rgba[:, :, 2] = disp
            rgba[:, :, 3] = 255
            rgba = np.flipud(rgba)

            tex = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, tex)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, rgba.tobytes())
            glEnable(GL_TEXTURE_2D)
            glColor4f(1, 1, 1, 1)
            glBegin(GL_QUADS)
            glTexCoord2f(0, 1); glVertex2f(x, y)
            glTexCoord2f(1, 1); glVertex2f(x + w, y)
            glTexCoord2f(1, 0); glVertex2f(x + w, y + h)
            glTexCoord2f(0, 0); glVertex2f(x, y + h)
            glEnd()
            glDisable(GL_TEXTURE_2D)
            glDeleteTextures([tex])

        # LED circle markers (visible vs occluded)
        if led_visible or led_occluded:
            sx = w / self.cam.width
            sy = h / self.cam.height
            radius = 5
            n_seg = 20
            # Visible LEDs: bright markers
            if led_visible:
                glColor4f(0.35, 0.75, 1.0, 0.9)
                glLineWidth(1.5)
                for (u, v, _d) in led_visible:
                    cx = x + u * sx
                    cy = y + v * sy
                    glBegin(GL_LINE_LOOP)
                    for k in range(n_seg):
                        angle = 2.0 * math.pi * k / n_seg
                        glVertex2f(
                            cx + radius * math.cos(angle),
                            cy + radius * math.sin(angle),
                        )
                    glEnd()
                glLineWidth(1)

            # Occluded LEDs: dimmer, smaller markers
            if led_occluded:
                glColor4f(0.35, 0.75, 1.0, 0.35)
                glLineWidth(1.0)
                occ_radius = max(2, radius - 2)
                for (u, v, _d) in led_occluded:
                    cx = x + u * sx
                    cy = y + v * sy
                    glBegin(GL_LINE_LOOP)
                    for k in range(n_seg):
                        angle = 2.0 * math.pi * k / n_seg
                        glVertex2f(
                            cx + occ_radius * math.cos(angle),
                            cy + occ_radius * math.sin(angle),
                        )
                    glEnd()
                glLineWidth(1)

        # Border frame (double line effect)
        glColor4f(*[c / 255.0 for c in Theme.IR_BORDER], 0.5)
        glLineWidth(3)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()
        glColor4f(*[c / 255.0 for c in Theme.IR_BORDER], 0.9)
        glLineWidth(1)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()

        # Label tab
        label_w = len(label) * 7 + 16
        label_h = 20
        glColor4f(*[c / 255.0 for c in Theme.IR_BORDER], 0.85)
        glBegin(GL_QUADS)
        glVertex2f(x, y - label_h); glVertex2f(x + label_w, y - label_h)
        glVertex2f(x + label_w, y); glVertex2f(x, y)
        glEnd()

        glDisable(GL_BLEND)
        self.text.draw(label, x + 8, y - label_h + 3, Theme.TITLE_TXT, "small")

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    # ------------------------------------------------------------------
    def _draw_ui(self):
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.win_w, self.win_h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)

        fps = self._publish_fps

        # ---- Top bar ----
        bar_h = 32
        self._draw_rect(0, 0, self.win_w, bar_h, Theme.TITLE_BG, 0.95)
        # Accent line at bottom of title bar
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(*[c / 255.0 for c in Theme.ACCENT], 0.6)
        glLineWidth(2)
        glBegin(GL_LINES)
        glVertex2f(0, bar_h); glVertex2f(self.win_w, bar_h)
        glEnd()
        glLineWidth(1)
        glDisable(GL_BLEND)

        self.text.draw("MOCAP SIM", 12, 8, Theme.ACCENT, "medium")
        self.text.draw(f"Frames {self.frames_sent}", 160, 10,
                       Theme.SUCCESS, "small")
        self.text.draw(
            f"{self.cam.width}x{self.cam.height}  "
            f"FOV {self.cam.fov:.0f}\u00b0  "
            f"BL {self.cam.baseline * 100:.1f}cm",
            340, 10, Theme.TEXT_DIM, "small"
        )

        fps_target = max(1, int(self.target_fps))
        if fps >= 0.9 * fps_target:
            fps_color = Theme.SUCCESS
        elif fps >= 0.5 * fps_target:
            fps_color = Theme.WARNING
        else:
            fps_color = Theme.ERROR
        self.text.draw(f"{fps:.0f}/{fps_target} FPS",
                       self.win_w - 110, 8, fps_color, "normal")

        # ---- Left panel (compact: status only, no help text) ----
        px, py = 8, 40
        pw = 260
        # Compute panel height dynamically
        y_cursor = py + 10
        section_h = (
            18 + 14 * 3 + 6    # MODE
            + 18 + 14 * 5 + 10  # CAMERAS (L/R/BL + axes + colors)
            + 18 + len(self.boards) * 56 + 4  # BOARDS
        )
        ph = section_h + 16
        ir_limit = self.win_h - 220 - 8  # don't overlap bottom IR previews
        ph = min(ph, ir_limit - py)

        self._draw_panel(px, py, pw, ph, alpha=0.88, accent_color=Theme.ACCENT)
        y = py + 10

        # Mode section
        self.text.draw("STATUS", px + 10, y, Theme.SECTION, "small"); y += 18
        traj = self.trajectory
        traj_clr = Theme.SUCCESS if traj.active else (Theme.WARNING if traj.waypoints else Theme.TEXT_DIM)
        self.text.draw(f"Traj: {traj.status_text}", px + 12, y, traj_clr, "small"); y += 14
        if self._preset_index >= 0:
            pname = PRESET_POSES[self._preset_index]["name"]
            self.text.draw(f"Preset: F{self._preset_index+1} {pname}",
                           px + 12, y, Theme.ACCENT, "small")
        else:
            self.text.draw("Preset: --", px + 12, y, Theme.TEXT_DIM, "small")
        y += 14
        st = "ON" if self.noise_gen.active else "OFF"
        sc = Theme.SUCCESS if self.noise_gen.active else Theme.TEXT_DIM
        self.text.draw(f"Noise: {st}", px + 12, y, sc, "small"); y += 16

        self._draw_separator(px + 10, y, pw - 20); y += 6

        # Camera section
        self.text.draw("CAMERAS", px + 10, y, Theme.SECTION, "small"); y += 18
        pL = self.cam.pos_left
        pR = self.cam.pos_right
        self.text.draw(f"L [{pL[0]:.3f}, {pL[1]:.3f}, {pL[2]:.3f}]",
                       px + 12, y, (80, 180, 255), "small"); y += 14
        self.text.draw(f"R [{pR[0]:.3f}, {pR[1]:.3f}, {pR[2]:.3f}]",
                       px + 12, y, (255, 170, 80), "small"); y += 14
        cam_center = (pL + pR) / 2.0
        self.text.draw(f"BL {self.cam.baseline*100:.2f}cm  FOV {self.cam.fov:.1f}\u00b0",
                       px + 12, y, Theme.TEXT_DIM, "small"); y += 16
        self.text.draw("Axes: X red, Y green, Z blue",
                       px + 12, y, Theme.TEXT_DIM, "small"); y += 14
        self.text.draw("L: blue frustum  R: orange frustum",
                       px + 12, y, Theme.TEXT_DIM, "small"); y += 16

        self._draw_separator(px + 10, y, pw - 20); y += 6

        # Boards section
        self.text.draw("BOARDS", px + 10, y, Theme.SECTION, "small"); y += 18
        for bi, board in enumerate(self.boards):
            if y + 50 > py + ph - 4:
                break
            sel = bi == self.selected_board
            if sel:
                self._draw_rect(px + 4, y - 2, pw - 8, 52, Theme.PANEL_ACC, 0.5)
            clr = Theme.ACCENT if sel else Theme.TEXT
            self.text.draw(f"{'>' if sel else ' '} {board.name}", px + 12, y, clr, "small"); y += 14
            p = board.position
            e = board.euler_angles
            self.text.draw(f"  XYZ [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]",
                           px + 12, y, Theme.TEXT_DIM, "small"); y += 13
            self.text.draw(f"  RPY [{e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f}]\u00b0",
                           px + 12, y, Theme.TEXT_DIM, "small"); y += 13
            dist = float(np.linalg.norm(p - cam_center))
            vis_L, vis_R, total = self._vis_stats[bi]
            dist_clr = Theme.SUCCESS if dist < 3.0 else (Theme.WARNING if dist < 5.0 else Theme.ERROR)
            self.text.draw(f"  {dist:.2f}m  L={vis_L}/{total} R={vis_R}/{total}",
                           px + 12, y, dist_clr, "small"); y += 16

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    # ------------------------------------------------------------------
    # Evaluation panel (right side)
    # ------------------------------------------------------------------
    def _draw_eval_panel(self):
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.win_w, self.win_h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)

        pw = 280
        px = self.win_w - pw - 8
        py = 40
        y = py + 10

        ev = self.evaluator
        has_data = bool(ev.errors)

        if ev.connected:
            conn_clr = Theme.SUCCESS
            conn_txt = f"Connected  (recv: {ev.recv_count})"
        else:
            conn_clr = Theme.ERROR
            conn_txt = "Waiting for C++ pipeline..."

        n_boards = len(ev.errors) if has_data else 0
        ph = 70 + n_boards * 155
        if not has_data:
            ph = 90

        self._draw_panel(px, py, pw, ph, alpha=0.93, accent_color=Theme.SUCCESS)

        self.text.draw("EVALUATION", px + 12, y, Theme.SECTION, "medium")
        y += 22
        self.text.draw(conn_txt, px + 16, y, conn_clr, "small")
        y += 22

        if has_data:
            for i, (name, err) in enumerate(ev.errors.items()):
                if i > 0:
                    self._draw_separator(px + 12, y, pw - 24); y += 8

                avg_pos = ev.get_avg_pos_error_mm(name)
                avg_rot = ev.get_avg_rot_error_deg(name)
                max_pos = ev.get_max_pos_error_mm(name)

                self.text.draw(name, px + 16, y, Theme.ACCENT, "small")
                y += 18

                ep = err.estimated_pos
                gp = err.gt_pos
                self.text.draw(
                    f"Est [{ep[0]:.3f},{ep[1]:.3f},{ep[2]:.3f}]",
                    px + 22, y, Theme.RECON, "small")
                y += 14
                self.text.draw(
                    f"GT  [{gp[0]:.3f},{gp[1]:.3f},{gp[2]:.3f}]",
                    px + 22, y, Theme.SUCCESS, "small")
                y += 16

                pe = err.pos_error_mm
                pe_clr = Theme.SUCCESS if pe < 5 else (Theme.WARNING if pe < 15 else Theme.ERROR)
                self._draw_rect(px + 16, y - 1, pw - 32, 15, pe_clr, 0.12)
                self.text.draw(f"Pos err: {pe:.1f} mm", px + 22, y, pe_clr, "small")
                y += 16

                dx, dy, dz = err.pos_error_xyz_mm
                self.text.draw(
                    f"  dX={dx:+.1f} dY={dy:+.1f} dZ={dz:+.1f}",
                    px + 22, y, Theme.TEXT_DIM, "small")
                y += 15

                re = err.rot_error_deg
                re_clr = Theme.SUCCESS if re < 2 else (Theme.WARNING if re < 5 else Theme.ERROR)
                self._draw_rect(px + 16, y - 1, pw - 32, 15, re_clr, 0.12)
                self.text.draw(f"Rot err: {re:.2f} deg", px + 22, y, re_clr, "small")
                y += 16

                dr, dp, dyw = err.rot_error_euler_deg
                self.text.draw(
                    f"  dR={dr:+.1f} dP={dp:+.1f} dY={dyw:+.1f}",
                    px + 22, y, Theme.TEXT_DIM, "small")
                y += 15

                self.text.draw(
                    f"Avg: {avg_pos:.1f}mm  Max: {max_pos:.1f}mm",
                    px + 22, y, Theme.TEXT_DIM, "small")
                y += 15

                self.text.draw(
                    f"RMSE: {err.rmse_mm:.1f}mm  Inliers: {err.num_inliers}",
                    px + 22, y, Theme.TEXT_DIM, "small")
                y += 20

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    @staticmethod
    def _draw_rect(x, y, w, h, color, alpha=1.0):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(*[c / 255.0 for c in color], alpha)
        glBegin(GL_QUADS)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_BLEND)

    @staticmethod
    def _draw_panel(x, y, w, h, alpha=0.95, accent_color=None, accent_h=3):
        """Panel with drop shadow, border, and optional top accent bar."""
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        # Shadow
        glColor4f(0.0, 0.0, 0.0, 0.3)
        glBegin(GL_QUADS)
        glVertex2f(x + 3, y + 3); glVertex2f(x + w + 3, y + 3)
        glVertex2f(x + w + 3, y + h + 3); glVertex2f(x + 3, y + h + 3)
        glEnd()
        # Background
        glColor4f(*[c / 255.0 for c in Theme.PANEL], alpha)
        glBegin(GL_QUADS)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()
        # Top accent bar
        if accent_color:
            glColor4f(*[c / 255.0 for c in accent_color], 1.0)
            glBegin(GL_QUADS)
            glVertex2f(x, y); glVertex2f(x + w, y)
            glVertex2f(x + w, y + accent_h); glVertex2f(x, y + accent_h)
            glEnd()
        # Border
        glColor4f(*[c / 255.0 for c in Theme.BORDER], 0.6)
        glLineWidth(1)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_BLEND)

    @staticmethod
    def _draw_separator(x, y, w):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(*[c / 255.0 for c in Theme.BORDER], 0.4)
        glLineWidth(1)
        glBegin(GL_LINES)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glEnd()
        glDisable(GL_BLEND)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        print("Simulation running.")
        print("  Left-drag: move board XY | Shift+Left-drag: rotate Pitch/Yaw")
        print("  Ctrl+Left-drag: rotate Roll | Ctrl+Scroll: move Z")
        print("  F1-F9: preset poses | P: trajectory playback | N: noise | V: eval panel")
        print("  WASD/QE: fine move | RFTGYH: fine rotate | Close window to exit.")

        self._running = True
        bg_thread = threading.Thread(target=self._publish_loop, daemon=True)
        bg_thread.start()

        try:
            while True:
                if not self._handle_input():
                    break

                # Trajectory playback drives board pose when active
                if self.trajectory.active and len(self.boards) > 0:
                    result = self.trajectory.step(self.boards[self.selected_board])
                    if result is not None:
                        pos, euler = result
                        board = self.boards[self.selected_board]
                        board.position = np.array(pos, dtype=np.float32)
                        board.euler_angles = np.array(euler, dtype=np.float64)
                        board.set_pose_euler(board.position, board.euler_angles)
                        with self._pose_lock:
                            self._occlusion_dirty = True

                self.evaluator.update()

                # Log trajectory errors
                if self.trajectory.active:
                    self.trajectory.log_frame(
                        self.frames_sent, self.evaluator, self.boards
                    )

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

                self._draw_god_view()

                with self._ir_lock:
                    img_L = self.ir_image_L
                    img_R = self.ir_image_R
                    px_L = self._led_px_L
                    px_R = self._led_px_R
                    px_L_occ = self._led_px_L_occ
                    px_R_occ = self._led_px_R_occ

                vw, vh = 300, 188
                margin = 12
                self._draw_ir_overlay(
                    img_L,
                    margin,
                    self.win_h - vh - margin,
                    vw,
                    vh,
                    "LEFT",
                    px_L,
                    px_L_occ,
                )
                self._draw_ir_overlay(
                    img_R,
                    self.win_w - vw - margin,
                    self.win_h - vh - margin,
                    vw,
                    vh,
                    "RIGHT",
                    px_R,
                    px_R_occ,
                )

                self._draw_ui()
                if self.show_eval:
                    self._draw_eval_panel()

                pygame.display.flip()
                self.clock.tick(self.target_fps + 10)  # allow slight headroom; publish thread controls actual rate

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            bg_thread.join(timeout=1.0)
            self._cleanup()

    def _cleanup(self):
        print(f"\nShutting down. Total frames sent: {self.frames_sent}")
        self.trajectory.stop_logging()
        self.evaluator.stop()
        self.text.clear()
        for dl in self._mesh_display_lists:
            if dl is not None:
                try:
                    glDeleteLists(dl, 1)
                except Exception:
                    pass
        self.pub_left.close()
        self.pub_right.close()
        self.zmq_ctx.term()
        pygame.quit()


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _dual_config_path(base_path: str) -> str:
    """Derive a dual-board config path from a base config path."""
    root, ext = os.path.splitext(os.path.abspath(base_path))
    if not ext:
        ext = ".json"
    return root + "_dual" + ext


def _ensure_dual_board_config(base_config: str) -> str:
    """
    Ensure a dual-board mocap_config JSON exists on disk.

    If ``<base>_dual.json`` already exists, it is reused. Otherwise, it is
    created by duplicating the first board entry in ``base_config``.
    """
    base_config = os.path.abspath(base_config)
    dual_path = _dual_config_path(base_config)

    if os.path.isfile(dual_path):
        print(f"[boards] Using existing dual-board config: {dual_path}")
        return dual_path

    try:
        with open(base_config, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except FileNotFoundError:
        print(f"[boards] WARNING: base config not found: {base_config}")
        print("          Dual-board mode falling back to single-board config.")
        return base_config

    boards = cfg.get("boards") or []
    if not boards:
        print(f"[boards] WARNING: no boards in base config {base_config}")
        return base_config

    if len(boards) >= 2:
        # Already multi-board; just reuse the same definition.
        out_cfg = {"boards": boards}
    else:
        b0 = boards[0]
        b1 = dict(b0)  # shallow copy is fine for JSON-serializable values
        name0 = b0.get("name", "Board")
        # Give the second board a distinguishable name while keeping it simple.
        b1["name"] = f"{name0}_R" if name0 else "Board_R"
        out_cfg = {"boards": [b0, b1]}

    os.makedirs(os.path.dirname(dual_path), exist_ok=True)
    with open(dual_path, "w", encoding="utf-8") as f:
        json.dump(out_cfg, f, indent=4, ensure_ascii=False)

    print(f"[boards] Dual-board config written to: {dual_path}")
    return dual_path


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    default_dir = os.path.dirname(os.path.abspath(__file__))

    parser = argparse.ArgumentParser(description="Mocap Simulation ZMQ Publisher")
    parser.add_argument("-c", "--config", default=os.path.join(default_dir, "config", "mocap_config.json"),
                        help="Board model JSON (default: config/mocap_config.json)")
    parser.add_argument("--calib", default=os.path.join(default_dir, "config", "calibration_sim.json"),
                        help="Calibration JSON (default: config/calibration_sim.json)")
    parser.add_argument("--port", type=int, default=5552,
                        help="ZMQ base port for left cam; right = port+1 (default: 5552)")
    parser.add_argument("--fps", type=int, default=60,
                        help="Target frame rate (default: 60)")
    parser.add_argument("--jpeg-quality", type=int, default=75,
                        help="JPEG encode quality 0-100 (default: 85)")
    parser.add_argument("--eval-port", type=int, default=5556,
                        help="ZMQ port for C++ pose output evaluation (default: 5556)")
    parser.add_argument(
        "--mesh",
        action="append",
        default=None,
        metavar="SPEC",
        help="Occlusion STL: 'IDX:path.stl' or path (uses board 0). Overrides JSON mesh_stl. Repeatable.",
    )
    parser.add_argument(
        "--points-3d",
        dest="points_3d_json",
        default=None,
        help="JSON with 'points_3d' in mm (optional point_ids) to override one board",
    )
    parser.add_argument(
        "--points-board",
        type=int,
        default=0,
        help="Board index for --points-3d (default: 0)",
    )
    parser.add_argument(
        "--no-occlusion",
        action="store_true",
        help="Disable ray-mesh occlusion even if mesh is configured",
    )
    parser.add_argument(
        "--occlusion-eps-mm",
        type=float,
        default=1.5,
        help="Ray hit tolerance vs LED depth (mm) to skip self-hits (default: 1.5)",
    )
    parser.add_argument(
        "--no-backface-cull",
        action="store_true",
        help="Disable LED surface back-face test (only ray occlusion)",
    )
    parser.add_argument(
        "--trajectory",
        default=None,
        help="Trajectory JSON file for automated pose sweep",
    )
    parser.add_argument(
        "--board-count",
        type=int,
        default=1,
        help="Number of simulated LED boards (1=default, 2=duplicate first board)",
    )
    args = parser.parse_args()

    # Optionally switch to a dual-board config synthesized from the base file.
    if args.board_count >= 2:
        args.config = _ensure_dual_board_config(args.config)

    app = SimPublisher(args)
    app.run()


if __name__ == "__main__":
    main()
