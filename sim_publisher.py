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
"""

import argparse
import math
import os
import struct
import sys
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
# UI Theme (dark)
# ---------------------------------------------------------------------------
class Theme:
    BG        = (18, 18, 24)
    PANEL     = (28, 28, 36)
    BORDER    = (60, 60, 80)
    TEXT      = (248, 250, 252)
    TEXT_DIM  = (148, 163, 184)
    ACCENT    = (99, 102, 241)
    SUCCESS   = (34, 197, 94)
    WARNING   = (251, 191, 36)
    ERROR     = (239, 68, 68)
    INFO      = (59, 130, 246)
    LED       = (255, 100, 100)
    RECON     = (100, 200, 255)
    NOISE_CLR = (255, 180, 100)
    SELECTED  = (255, 215, 0)
    GRID_MAJ  = (55, 55, 70)
    GRID_MIN  = (35, 35, 45)


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
        self._led_px_L = []  # [(u,v,d), ...] for left camera overlay
        self._led_px_R = []  # [(u,v,d), ...] for right camera overlay

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
        self.dragging = False
        self._toggle_held = set()  # debounce for toggle keys (N, V)

        # --- ZMQ send status ---
        self.frames_sent = 0

        # --- Evaluator (subscribe to C++ pipeline output) ---
        eval_addr = f"tcp://localhost:{args.eval_port}"
        self.evaluator = PoseEvaluator(self.boards, address=eval_addr)
        self.evaluator.start()
        self.show_eval = True
        print(f"Eval SUB connecting to {eval_addr}")

        if self.occlusion.has_any_mesh() and not args.no_occlusion:
            print("Occlusion: ON (STL ray cast + back-face cull)")
        elif args.no_occlusion:
            print("Occlusion: OFF (--no-occlusion)")
        else:
            print("Occlusion: no mesh (use JSON mesh_stl or --mesh PATH)")

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

            elif ev.type == MOUSEBUTTONDOWN:
                if ev.button == 1:
                    self.dragging = True
                elif ev.button == 4:
                    self.cam_zoom += 0.2
                elif ev.button == 5:
                    self.cam_zoom -= 0.2

            elif ev.type == MOUSEBUTTONUP:
                if ev.button == 1:
                    self.dragging = False

            elif ev.type == MOUSEMOTION:
                if self.dragging:
                    dx, dy = ev.rel
                    self.cam_rot[1] += dx * 0.5
                    self.cam_rot[0] += dy * 0.5

            elif ev.type == KEYUP:
                self._toggle_held.discard(ev.key)

            elif ev.type == KEYDOWN:
                k = ev.key
                if k == K_v and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.show_eval = not self.show_eval
                elif k == K_n and k not in self._toggle_held:
                    self._toggle_held.add(k)
                    self.noise_gen.toggle()
                elif k == K_1:
                    self.selected_board = 0
                elif k == K_2 and len(self.boards) > 1:
                    self.selected_board = 1
                elif len(self.boards) > 0:
                    board = self.boards[self.selected_board]
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
                    board.set_pose_euler(board.position, board.euler_angles)
        return True

    # ------------------------------------------------------------------
    # Vision + ZMQ publish
    # ------------------------------------------------------------------
    def _process_and_publish(self):
        self.noise_gen.update()
        noise_pts = self.noise_gen.get_noise_world_points()

        all_pts = []
        for board in self.boards:
            all_pts.append(board.get_world_points())
        if len(all_pts):
            all_world = np.vstack(all_pts)
        else:
            all_world = np.empty((0, 3))
        n_board_leds = len(all_world)

        vis_L, vis_R = self.occlusion.visibility_for_board_leds(
            self.boards, self.cam.pos_left, self.cam.pos_right
        )
        self._last_vis_L = vis_L
        self._last_vis_R = vis_R
        o = 0
        for bi, board in enumerate(self.boards):
            n = len(board.get_world_points())
            self._vis_stats[bi] = (
                int(vis_L[o : o + n].sum()),
                int(vis_R[o : o + n].sum()),
                n,
            )
            o += n

        if len(noise_pts):
            all_world = np.vstack((all_world, noise_pts)) if len(all_world) else noise_pts

        noise_n = len(noise_pts)
        vis_L_ext = (
            np.concatenate([vis_L, np.ones(noise_n, dtype=bool)])
            if noise_n
            else vis_L
        )
        vis_R_ext = (
            np.concatenate([vis_R, np.ones(noise_n, dtype=bool)])
            if noise_n
            else vis_R
        )

        led_L = self.cam.project_with_distance(all_world, is_left=True)
        led_R = self.cam.project_with_distance(all_world, is_left=False)

        led_L = [p if (p is not None and v) else None for p, v in zip(led_L, vis_L_ext)]
        led_R = [p if (p is not None and v) else None for p, v in zip(led_R, vis_R_ext)]

        valid_L = [p for p in led_L if p is not None]
        valid_R = [p for p in led_R if p is not None]
        self._led_px_L = list(valid_L)
        self._led_px_R = list(valid_R)

        noise_on = self.noise_gen.active
        self.ir_image_L = self.ir_gen_L.generate_image(valid_L, noise_on)
        self.ir_image_R = self.ir_gen_R.generate_image(valid_R, noise_on)

        ts_ns = int(time.time() * 1e9)
        header = struct.pack("<Q", ts_ns)
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        ok_l, jpg_l = cv2.imencode(".jpg", self.ir_image_L, encode_params)
        ok_r, jpg_r = cv2.imencode(".jpg", self.ir_image_R, encode_params)

        if ok_l:
            self.pub_left.send(header + jpg_l.tobytes(), zmq.NOBLOCK)
        if ok_r:
            self.pub_right.send(header + jpg_r.tobytes(), zmq.NOBLOCK)

        self.frames_sent += 1

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
        glBegin(GL_LINES)
        glColor4f(*[c / 255.0 for c in Theme.GRID_MIN], 0.5)
        for i in range(-10, 11):
            if i % 5 != 0:
                glVertex3f(-2, i * 0.2, -0.5); glVertex3f(2, i * 0.2, -0.5)
                glVertex3f(i * 0.2, -2, -0.5); glVertex3f(i * 0.2, 2, -0.5)
        glEnd()
        glBegin(GL_LINES)
        glColor4f(*[c / 255.0 for c in Theme.GRID_MAJ], 0.8)
        for i in range(-2, 3):
            glVertex3f(-2, i, -0.5); glVertex3f(2, i, -0.5)
            glVertex3f(i, -2, -0.5); glVertex3f(i, 2, -0.5)
        glEnd()

        # Axes
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(2)
        glBegin(GL_LINES)
        glColor4f(*[c / 255.0 for c in Theme.ERROR], 0.8)
        glVertex3f(0, 0, 0); glVertex3f(0.5, 0, 0)
        glColor4f(*[c / 255.0 for c in Theme.SUCCESS], 0.8)
        glVertex3f(0, 0, 0); glVertex3f(0, 0.5, 0)
        glColor4f(*[c / 255.0 for c in Theme.INFO], 0.8)
        glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.5)
        glEnd()
        glLineWidth(1)
        glDisable(GL_BLEND)

        # Cameras (small frustums with L/R labels)
        for pos, label, clr in [
            (self.cam.pos_left,  "L", (0.3, 0.8, 1.0)),
            (self.cam.pos_right, "R", (1.0, 0.6, 0.3)),
        ]:
            glColor3f(*clr)
            glPushMatrix()
            glTranslatef(*pos)
            self._draw_camera_body()
            glPopMatrix()

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
        s = 0.015
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
    def _draw_ir_overlay(self, ir_img, x, y, w, h, label, led_positions=None):
        """Draw a small IR image preview with blue circle markers on LEDs."""
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.win_w, self.win_h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)

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

        # Blue circle markers for detected LEDs
        if led_positions:
            sx = w / self.cam.width
            sy = h / self.cam.height
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0.4, 0.7, 1.0, 1.0)
            glLineWidth(1.5)
            radius = 5
            n_seg = 20
            for (u, v, _d) in led_positions:
                cx = x + u * sx
                cy = y + v * sy
                glBegin(GL_LINE_LOOP)
                for k in range(n_seg):
                    angle = 2.0 * math.pi * k / n_seg
                    glVertex2f(cx + radius * math.cos(angle),
                               cy + radius * math.sin(angle))
                glEnd()
            glLineWidth(1)
            glDisable(GL_BLEND)

        # Border
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(*[c / 255.0 for c in Theme.SUCCESS], 0.8)
        glLineWidth(2)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y); glVertex2f(x + w, y)
        glVertex2f(x + w, y + h); glVertex2f(x, y + h)
        glEnd()
        glLineWidth(1)
        glDisable(GL_BLEND)

        self.text.draw(label, x + 6, y + 4, Theme.TEXT, "small")

        glEnable(GL_DEPTH_TEST)
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

        fps = self.clock.get_fps()

        # Top bar
        self._draw_rect(10, 10, self.win_w - 20, 36, Theme.PANEL, 0.9)
        self.text.draw("MOCAP SIM PUBLISHER", 20, 16, Theme.TEXT, "medium")
        self.text.draw(f"ZMQ  Frames: {self.frames_sent}", 250, 18, Theme.SUCCESS, "small")
        self.text.draw(
            f"Cam: {self.cam.width}x{self.cam.height}  "
            f"FOV: {self.cam.fov:.0f}°  "
            f"BL: {self.cam.baseline * 100:.1f}cm",
            500, 18, Theme.TEXT_DIM, "small"
        )
        fps_color = Theme.SUCCESS if fps > 25 else (Theme.WARNING if fps > 15 else Theme.ERROR)
        self.text.draw(f"FPS: {fps:.0f}", self.win_w - 100, 18, fps_color, "normal")

        # Left panel – controls + info
        px, py = 10, 56
        pw = 280

        # Calculate panel content height dynamically
        content_lines = 6 + 3 + 3 + len(self.boards) * 5 + 4  # controls + noise + cameras + boards + distance
        ph = max(340, content_lines * 16 + 40)
        self._draw_rect(px, py, pw, ph, Theme.PANEL, 0.85)
        y = py + 12

        self.text.draw("CONTROLS", px + 12, y, Theme.INFO, "medium"); y += 22
        for key, desc in [
            ("1/2", "Select Board"), ("WASD+QE", "Move Board"),
            ("R/F T/G Y/H", "Rotate (RPY)"), ("Mouse Drag", "Orbit View"),
            ("Scroll", "Zoom"), ("N", "Toggle Noise"),
            ("V", "Toggle Eval Panel"),
        ]:
            self.text.draw(key, px + 14, y, Theme.ACCENT, "small")
            self.text.draw(desc, px + 105, y, Theme.TEXT_DIM, "small")
            y += 18
        y += 6

        # Noise status
        self.text.draw("NOISE", px + 12, y, Theme.WARNING, "medium"); y += 20
        st = "ON" if self.noise_gen.active else "OFF"
        sc = Theme.SUCCESS if self.noise_gen.active else Theme.ERROR
        self.text.draw(f"Status: {st}", px + 14, y, sc, "small"); y += 20

        # Camera positions
        self.text.draw("CAMERAS", px + 12, y, Theme.INFO, "medium"); y += 20
        pL = self.cam.pos_left
        pR = self.cam.pos_right
        self.text.draw(f"L [{pL[0]:.3f}, {pL[1]:.3f}, {pL[2]:.3f}]",
                       px + 14, y, (77, 204, 255), "small"); y += 15
        self.text.draw(f"R [{pR[0]:.3f}, {pR[1]:.3f}, {pR[2]:.3f}]",
                       px + 14, y, (255, 153, 77), "small"); y += 15
        cam_center = (pL + pR) / 2.0
        self.text.draw(f"BL: {self.cam.baseline*100:.2f}cm  FOV: {self.cam.fov:.1f}\u00b0",
                       px + 14, y, Theme.TEXT_DIM, "small"); y += 20

        # Board info
        self.text.draw("BOARDS", px + 12, y, Theme.INFO, "medium"); y += 20
        for bi, board in enumerate(self.boards):
            sel = bi == self.selected_board
            clr = Theme.SELECTED if sel else Theme.TEXT_DIM
            self.text.draw(f"{'>' if sel else ' '} {board.name}", px + 14, y, clr, "small"); y += 16
            p = board.position
            e = board.euler_angles
            self.text.draw(f"  Pos [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]",
                           px + 14, y, Theme.TEXT_DIM, "small"); y += 14
            self.text.draw(f"  Rot [{e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f}]\u00b0",
                           px + 14, y, Theme.TEXT_DIM, "small"); y += 14
            dist = float(np.linalg.norm(p - cam_center))
            vis_L, vis_R, total = self._vis_stats[bi]
            dist_clr = Theme.SUCCESS if dist < 3.0 else (Theme.WARNING if dist < 5.0 else Theme.ERROR)
            self.text.draw(f"  Dist: {dist:.2f}m  Vis: L={vis_L}/{total} R={vis_R}/{total}",
                           px + 14, y, dist_clr, "small"); y += 18

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

        pw = 300
        px = self.win_w - pw - 10
        py = 56
        y = py + 12

        ev = self.evaluator
        has_data = bool(ev.errors)

        # Connection status
        if ev.connected:
            conn_clr = Theme.SUCCESS
            conn_txt = f"Connected  (recv: {ev.recv_count})"
        else:
            conn_clr = Theme.ERROR
            conn_txt = "Waiting for C++ pipeline..."

        # Calculate panel height
        n_boards = len(ev.errors) if has_data else 0
        ph = 60 + n_boards * 145
        if not has_data:
            ph = 80

        self._draw_rect(px, py, pw, ph, Theme.PANEL, 0.88)

        self.text.draw("EVALUATION", px + 12, y, (255, 120, 200), "medium")
        y += 22
        self.text.draw(conn_txt, px + 14, y, conn_clr, "small")
        y += 20

        if has_data:
            for name, err in ev.errors.items():
                avg_pos = ev.get_avg_pos_error_mm(name)
                avg_rot = ev.get_avg_rot_error_deg(name)
                max_pos = ev.get_max_pos_error_mm(name)

                self.text.draw(name, px + 14, y, Theme.ACCENT, "small")
                y += 18

                # Raw positions for verification
                ep = err.estimated_pos
                gp = err.gt_pos
                self.text.draw(
                    f"Est [{ep[0]:.3f},{ep[1]:.3f},{ep[2]:.3f}]",
                    px + 20, y, Theme.RECON, "small")
                y += 14
                self.text.draw(
                    f"GT  [{gp[0]:.3f},{gp[1]:.3f},{gp[2]:.3f}]",
                    px + 20, y, Theme.SUCCESS, "small")
                y += 16

                # Position error with color coding
                pe = err.pos_error_mm
                pe_clr = Theme.SUCCESS if pe < 5 else (Theme.WARNING if pe < 15 else Theme.ERROR)
                self.text.draw(f"Pos err: {pe:.1f} mm", px + 20, y, pe_clr, "small")
                y += 15

                dx, dy, dz = err.pos_error_xyz_mm
                self.text.draw(
                    f"  dX={dx:+.1f} dY={dy:+.1f} dZ={dz:+.1f}",
                    px + 20, y, Theme.TEXT_DIM, "small")
                y += 15

                # Rotation error
                re = err.rot_error_deg
                re_clr = Theme.SUCCESS if re < 2 else (Theme.WARNING if re < 5 else Theme.ERROR)
                self.text.draw(f"Rot err: {re:.2f} deg", px + 20, y, re_clr, "small")
                y += 15

                dr, dp, dyw = err.rot_error_euler_deg
                self.text.draw(
                    f"  dR={dr:+.1f} dP={dp:+.1f} dY={dyw:+.1f}",
                    px + 20, y, Theme.TEXT_DIM, "small")
                y += 15

                # Averages
                self.text.draw(
                    f"Avg: {avg_pos:.1f}mm  Max: {max_pos:.1f}mm",
                    px + 20, y, Theme.TEXT_DIM, "small")
                y += 15

                # C++ pipeline RMSE and inliers
                self.text.draw(
                    f"RMSE: {err.rmse_mm:.1f}mm  Inliers: {err.num_inliers}",
                    px + 20, y, Theme.TEXT_DIM, "small")
                y += 18

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

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        print("Simulation running. Move boards with WASD/QE, rotate with RFTGYH.")
        print("Press N for noise toggle. Close window to exit.")

        try:
            while True:
                if not self._handle_input():
                    break

                self._process_and_publish()
                self.evaluator.update()

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

                self._draw_god_view()

                vw, vh = 320, 200
                margin = 20
                self._draw_ir_overlay(
                    self.ir_image_L,
                    margin, self.win_h - vh - margin, vw, vh, "LEFT CAM",
                    self._led_px_L,
                )
                self._draw_ir_overlay(
                    self.ir_image_R,
                    self.win_w - vw - margin, self.win_h - vh - margin, vw, vh, "RIGHT CAM",
                    self._led_px_R,
                )

                self._draw_ui()
                if self.show_eval:
                    self._draw_eval_panel()

                pygame.display.flip()
                self.clock.tick(self.target_fps)

        except KeyboardInterrupt:
            pass
        finally:
            self._cleanup()

    def _cleanup(self):
        print(f"\nShutting down. Total frames sent: {self.frames_sent}")
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
    parser.add_argument("--fps", type=int, default=30,
                        help="Target frame rate (default: 30)")
    parser.add_argument("--jpeg-quality", type=int, default=95,
                        help="JPEG encode quality 0-100 (default: 95)")
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
    args = parser.parse_args()

    app = SimPublisher(args)
    app.run()


if __name__ == "__main__":
    main()
