"""
Real-time pose evaluation: subscribes to C++ pipeline output via ZMQ,
compares estimated poses against simulation ground truth, and computes
per-board error metrics.

The C++ pipeline outputs poses in FLU body frame (origin = midpoint of
stereo cameras), which is the same frame used by the simulator.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict

import numpy as np
import zmq


@dataclass
class BoardError:
    """Per-board error metrics for a single frame."""
    board_name: str = ""
    pos_error_mm: float = 0.0
    pos_error_xyz_mm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rot_error_deg: float = 0.0
    rot_error_euler_deg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    latency_ms: float = 0.0
    num_inliers: int = 0
    rmse_mm: float = 0.0
    estimated_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gt_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    timestamp: float = 0.0


class _EvalReceiver(threading.Thread):
    """Background thread that subscribes to C++ pose output."""

    def __init__(self, address: str):
        super().__init__(daemon=True)
        self.address = address
        self._lock = threading.Lock()
        self._latest = None
        self._running = True
        self._connected = False
        self._recv_count = 0

    def run(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.setsockopt(zmq.RCVHWM, 5)
        try:
            sock.connect(self.address)
            self._connected = True
        except zmq.ZMQError as exc:
            print(f"[Eval] ZMQ connect failed: {exc}")
            return

        while self._running:
            try:
                msg = sock.recv_string()
                data = json.loads(msg)
                data["_recv_time"] = time.time()
                with self._lock:
                    self._latest = data
                    self._recv_count += 1
            except zmq.Again:
                continue
            except Exception:
                continue

        sock.close()
        ctx.term()

    def get_latest(self):
        with self._lock:
            data = self._latest
            self._latest = None
            return data

    @property
    def connected(self):
        return self._connected

    @property
    def recv_count(self):
        return self._recv_count

    def stop(self):
        self._running = False


HISTORY_LEN = 100


class PoseEvaluator:
    """Compares C++ pipeline poses against simulation GT in real-time."""

    def __init__(self, boards, address="tcp://localhost:5556"):
        self._boards = boards
        self._receiver = _EvalReceiver(address)
        self.errors: Dict[str, BoardError] = {}
        self._history: Dict[str, deque] = {}
        self._active = False
        self.frame_id = -1

    def start(self):
        self._receiver.start()
        self._active = True

    def stop(self):
        self._receiver.stop()
        self._active = False

    @property
    def connected(self):
        return self._receiver.connected

    @property
    def recv_count(self):
        return self._receiver.recv_count

    def update(self):
        """Poll for new C++ results and compute errors against current GT."""
        if not self._active:
            return

        data = self._receiver.get_latest()
        if data is None:
            return

        self.frame_id = data.get("frame_id", -1)
        recv_time = data.get("_recv_time", time.time())
        ts = data.get("timestamp", 0.0)

        gt_map = {b.name: b for b in self._boards}

        for pose_data in data.get("poses", []):
            board_name = pose_data.get("board_name", "")
            gt = gt_map.get(board_name)
            if gt is None:
                continue

            est_pos = np.array(pose_data["position"], dtype=np.float64)
            gt_pos = np.array(gt.position, dtype=np.float64)

            est_rot = np.array(pose_data["rotation"], dtype=np.float64)
            gt_rot = np.array(gt.rotation, dtype=np.float64)

            pos_diff = est_pos - gt_pos
            pos_err_mm = float(np.linalg.norm(pos_diff)) * 1000.0

            rot_err_deg = _rotation_error_deg(est_rot, gt_rot)

            est_euler = np.array(pose_data.get("euler_angles", [0, 0, 0]),
                                 dtype=np.float64)
            gt_euler = np.array(gt.euler_angles, dtype=np.float64)
            euler_diff = _wrap_angles(est_euler - gt_euler)

            latency = (recv_time - ts) * 1000.0 if ts > 1e9 else 0.0

            err = BoardError(
                board_name=board_name,
                pos_error_mm=pos_err_mm,
                pos_error_xyz_mm=pos_diff * 1000.0,
                rot_error_deg=rot_err_deg,
                rot_error_euler_deg=euler_diff,
                latency_ms=latency,
                num_inliers=pose_data.get("num_inliers", 0),
                rmse_mm=pose_data.get("rmse", 0.0) * 1000.0,
                estimated_pos=est_pos,
                gt_pos=gt_pos,
                timestamp=ts,
            )

            self.errors[board_name] = err

            if board_name not in self._history:
                self._history[board_name] = deque(maxlen=HISTORY_LEN)
            self._history[board_name].append(err)

    def get_avg_pos_error_mm(self, board_name: str) -> float:
        hist = self._history.get(board_name)
        if not hist:
            return 0.0
        return float(np.mean([e.pos_error_mm for e in hist]))

    def get_avg_rot_error_deg(self, board_name: str) -> float:
        hist = self._history.get(board_name)
        if not hist:
            return 0.0
        return float(np.mean([e.rot_error_deg for e in hist]))

    def get_max_pos_error_mm(self, board_name: str) -> float:
        hist = self._history.get(board_name)
        if not hist:
            return 0.0
        return float(np.max([e.pos_error_mm for e in hist]))


def _rotation_error_deg(R_est: np.ndarray, R_gt: np.ndarray) -> float:
    """Angle (degrees) between two rotation matrices."""
    R_err = R_est.T @ R_gt
    trace = np.clip(np.trace(R_err), -1.0, 3.0)
    angle_rad = math.acos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(angle_rad)


def _wrap_angles(angles_deg: np.ndarray) -> np.ndarray:
    """Wrap angle differences to [-180, 180]."""
    return (angles_deg + 180.0) % 360.0 - 180.0
