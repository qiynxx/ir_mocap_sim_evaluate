"""
Occlusion engine: STL mesh loading, ray-mesh intersection, and back-face culling.

Uses trimesh for ray casting against the board's 3D mesh to determine
which LEDs are visible from each camera. When no mesh is configured,
all LEDs are treated as visible (legacy behaviour).
"""

import math
import numpy as np

try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False


def _euler_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Build the *inverse* rotation used by the design-tool exporter."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return (Rz @ Ry @ Rx).T


class _BoardMesh:
    """Loaded mesh + cached data for one board."""

    __slots__ = ("mesh", "scale", "face_normals", "led_face_normals")

    def __init__(self, mesh: "trimesh.Trimesh", scale: float):
        self.mesh = mesh
        self.scale = scale
        self.face_normals = np.asarray(mesh.face_normals, dtype=np.float64)
        self.led_face_normals = None  # set after LED points are known


class OcclusionEngine:
    """
    Per-frame visibility computation for board LEDs.

    For each LED the engine performs:
      1. **Ray-mesh intersection** – cast a ray from the camera to the LED;
         if the nearest hit is closer than the LED (minus a tolerance), the
         LED is occluded.
      2. **Back-face culling** – if the surface normal at the closest face
         to the LED points away from the camera, the LED is not visible.

    Parameters
    ----------
    boards : list[BoardObject]
        Board objects with optional ``mesh_stl_path`` and ``mesh_units``.
    enabled : bool
        Master switch; when False the engine always returns all-visible.
    eps_m : float
        Depth tolerance (metres) for self-hit suppression.
    backface_cull : bool
        Whether to apply the surface-normal back-face test.
    """

    def __init__(self, boards, *, enabled=True, eps_m=0.0015,
                 backface_cull=True):
        self._enabled = enabled
        self._eps = eps_m
        self._backface = backface_cull
        self._meshes: list[_BoardMesh | None] = []

        for board in boards:
            self._meshes.append(self._load(board))

    @staticmethod
    def _load(board) -> "_BoardMesh | None":
        path = getattr(board, "mesh_stl_path", None)
        if not path:
            return None
        if not HAS_TRIMESH:
            print("[occlusion] trimesh not installed – skipping mesh")
            return None

        try:
            mesh = trimesh.load(path, force="mesh")
            if isinstance(mesh, trimesh.Scene):
                geoms = list(mesh.geometry.values())
                if not geoms:
                    print(f"[occlusion] empty scene: {path}")
                    return None
                mesh = geoms[0]
        except Exception as exc:
            print(f"[occlusion] failed to load {path}: {exc}")
            return None

        # --- Transform STL world coords → LED local coords (in mm) ---
        origin = getattr(board, "mesh_origin_world_mm", np.zeros(3))
        origin = np.asarray(origin, dtype=np.float64)
        axis_rot = getattr(board, "mesh_axis_rotation", [0, 0, 0])

        if np.any(origin != 0):
            mesh.vertices = mesh.vertices - origin
            print(f"[occlusion] mesh shifted by origin {origin.tolist()}")

        if any(a != 0 for a in axis_rot):
            R_inv = _euler_rotation_matrix(*axis_rot)
            mesh.vertices = (R_inv @ mesh.vertices.T).T
            print(f"[occlusion] mesh rotated by {axis_rot}")

        # --- Scale to metres ---
        units = getattr(board, "mesh_units", "mm")
        scale = {"mm": 0.001, "m": 1.0, "cm": 0.01}.get(units, 0.001)
        mesh.vertices *= scale
        mesh.fix_normals()

        print(f"[occlusion] loaded {path}  "
              f"verts={len(mesh.vertices)}  faces={len(mesh.faces)}  "
              f"units={units}→m  "
              f"bounds=[{mesh.bounds[0].tolist()}, {mesh.bounds[1].tolist()}]")
        return _BoardMesh(mesh, scale)

    def has_any_mesh(self) -> bool:
        return any(m is not None for m in self._meshes)

    def _precompute_led_normals(self, boards):
        """Cache the surface normal at each LED (nearest face in local frame).
        Called once so the expensive nearest.on_surface is not repeated."""
        for bi, board in enumerate(boards):
            bm = self._meshes[bi] if bi < len(self._meshes) else None
            if bm is None or bm.led_face_normals is not None:
                continue
            leds_local = np.asarray(board.model_points, dtype=np.float64)
            try:
                _, _, face_ids = bm.mesh.nearest.on_surface(leds_local)
                bm.led_face_normals = bm.face_normals[face_ids].copy()
            except Exception:
                bm.led_face_normals = np.zeros((len(leds_local), 3))

    def get_mesh_data(self, board_index: int):
        """
        Return mesh geometry for OpenGL rendering.

        Returns dict with 'vertices' (Nx3 float64, in metres, local frame),
        'faces' (Mx3 int), 'face_normals' (Mx3 float64), or None.
        """
        if board_index >= len(self._meshes):
            return None
        bm = self._meshes[board_index]
        if bm is None:
            return None
        return {
            "vertices": np.asarray(bm.mesh.vertices, dtype=np.float64),
            "faces": np.asarray(bm.mesh.faces, dtype=np.int32),
            "face_normals": bm.face_normals,
        }

    def visibility_for_board_leds(self, boards, cam_pos_left, cam_pos_right):
        """
        Compute per-LED visibility masks for left and right cameras.

        Rays are transformed into the mesh's local frame so the cached BVH
        is reused every frame. LED surface normals are pre-computed once.

        Returns
        -------
        vis_L, vis_R : np.ndarray[bool]
            Concatenated across all boards.
        """
        self._precompute_led_normals(boards)

        all_vis_L = []
        all_vis_R = []

        for bi, board in enumerate(boards):
            pts_w = board.get_world_points()
            n = len(pts_w)
            bmesh = self._meshes[bi] if bi < len(self._meshes) else None

            if (not self._enabled) or bmesh is None:
                all_vis_L.append(np.ones(n, dtype=bool))
                all_vis_R.append(np.ones(n, dtype=bool))
                continue

            R = board.rotation.astype(np.float64)
            R_inv = R.T
            pos = board.position.astype(np.float64)

            cam_L_local = R_inv @ (np.asarray(cam_pos_left, dtype=np.float64) - pos)
            cam_R_local = R_inv @ (np.asarray(cam_pos_right, dtype=np.float64) - pos)
            leds_local = np.asarray(board.model_points, dtype=np.float64)

            vl = self._check_visibility_local(
                leds_local, cam_L_local, bmesh)
            vr = self._check_visibility_local(
                leds_local, cam_R_local, bmesh)
            all_vis_L.append(vl)
            all_vis_R.append(vr)

        return (
            np.concatenate(all_vis_L) if all_vis_L else np.empty(0, dtype=bool),
            np.concatenate(all_vis_R) if all_vis_R else np.empty(0, dtype=bool),
        )

    def _check_visibility_local(self, led_pts_local, cam_local, bmesh):
        """
        Check visibility entirely in the mesh's local frame.
        The mesh's BVH is reused without rebuilding.
        Back-face culling uses pre-computed per-LED surface normals.
        """
        n = len(led_pts_local)
        vis = np.ones(n, dtype=bool)
        mesh = bmesh.mesh

        cam = np.asarray(cam_local, dtype=np.float64)
        origins = np.tile(cam, (n, 1))
        dirs = led_pts_local - cam
        dists = np.linalg.norm(dirs, axis=1, keepdims=True)
        dists_flat = dists.ravel()
        dirs_norm = dirs / np.clip(dists, 1e-12, None)

        try:
            locs, ray_idx, _ = mesh.ray.intersects_location(
                ray_origins=origins,
                ray_directions=dirs_norm,
                multiple_hits=True,
            )
        except Exception:
            return vis

        if len(locs) > 0:
            hit_dists = np.linalg.norm(locs - origins[ray_idx], axis=1)
            for i in range(n):
                mask = ray_idx == i
                if not mask.any():
                    continue
                nearest = hit_dists[mask].min()
                if nearest < dists_flat[i] - self._eps:
                    vis[i] = False

        # Back-face culling using cached per-LED normals (no on_surface call)
        if self._backface and bmesh.led_face_normals is not None:
            normals = bmesh.led_face_normals
            for i in range(n):
                if not vis[i]:
                    continue
                view_dir = cam - led_pts_local[i]
                if np.dot(normals[i], view_dir) <= 0:
                    vis[i] = False

        return vis
