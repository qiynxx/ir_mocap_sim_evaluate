"""
Simulation stereo camera that reads parameters from a Kalibr-format
calibration JSON (the same format used by mocap_ir_cpp).

Coordinate systems:

    World (FLU, right-handed):
        X = forward,  Y = left,  Z = up

    Camera / Sensor (OpenCV):
        X = right,  Y = down,  Z = forward (depth)

    Kalibr T_BS:
        Transforms a point FROM the body frame TO the sensor frame.
        When T_BS_left = Identity the body frame coincides with the
        left camera's sensor frame.

Projection chain for each camera:
    p_sensor = R_BS @ R_WORLD_TO_CAM @ p_world + t_BS

    where R_WORLD_TO_CAM is the fixed FLU→OpenCV rotation and
    (R_BS, t_BS) come from T_BS.
"""

import json
import math
import numpy as np

# Fixed rotation: world FLU → OpenCV camera frame
R_WORLD_TO_CAM = np.array([
    [0, -1,  0],   # X_cam = -Y_world  (right = -left)
    [0,  0, -1],   # Y_cam = -Z_world  (down  = -up)
    [1,  0,  0],   # Z_cam =  X_world  (fwd   =  fwd)
], dtype=np.float64)


class SimStereoCamera:
    """Ideal pinhole stereo camera loaded from a Kalibr calibration JSON."""

    def __init__(self, calib_path,
                 left_key="cam2_ov9281_0",
                 right_key="cam3_ov9281_1"):
        with open(calib_path, "r") as f:
            calib = json.load(f)

        cameras = calib["cameras"]
        left_cam = cameras[left_key]
        right_cam = cameras[right_key]

        # Resolution (from left camera)
        self.width = left_cam["resolution"][0]
        self.height = left_cam["resolution"][1]

        # Intrinsics
        fx, fy, cx, cy = left_cam["intrinsics"]
        self.K = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1],
        ], dtype=np.float64)
        self.focal_length = fx
        self.fov = math.degrees(2.0 * math.atan(self.width / (2.0 * fx)))

        # Build the full world→sensor transforms for each camera.
        T_left  = np.array(left_cam["T_BS"]["data"],  dtype=np.float64).reshape(4, 4)
        T_right = np.array(right_cam["T_BS"]["data"], dtype=np.float64).reshape(4, 4)

        R_BS_left,  t_BS_left  = T_left[:3, :3],  T_left[:3, 3]
        R_BS_right, t_BS_right = T_right[:3, :3], T_right[:3, 3]

        # Full rotation: world → sensor  =  R_BS @ R_WORLD_TO_CAM
        self._R_left  = R_BS_left  @ R_WORLD_TO_CAM
        self._R_right = R_BS_right @ R_WORLD_TO_CAM
        # Translation part of body→sensor (applied after rotating to body frame)
        self._t_left  = t_BS_left
        self._t_right = t_BS_right

        # Camera positions in FLU world (for OpenGL rendering).
        # p_sensor = R_full @ p_world + t_full => at camera center p_sensor=0:
        #   cam_world = -R_full^T @ t_full
        self.pos_left  = -self._R_left.T  @ self._t_left
        self.pos_right = -self._R_right.T @ self._t_right

        self.baseline = float(np.linalg.norm(self.pos_right - self.pos_left))

    # ------------------------------------------------------------------
    # Projection
    # ------------------------------------------------------------------

    def project_with_distance(self, points_3d, is_left=True):
        """
        Project world 3D points to 2D pixel coords with depth info.

        Args:
            points_3d: (N, 3) array in FLU world coordinates.
            is_left: True for left camera, False for right.

        Returns:
            List of (u, v, depth) or None per point.
        """
        R = self._R_left if is_left else self._R_right
        t = self._t_left if is_left else self._t_right
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        results = []
        for pt in np.asarray(points_3d):
            p_cam = R @ pt + t
            if p_cam[2] <= 0.01:
                results.append(None)
                continue
            u = fx * p_cam[0] / p_cam[2] + cx
            v = fy * p_cam[1] / p_cam[2] + cy
            if 0 <= u < self.width and 0 <= v < self.height:
                results.append((u, v, p_cam[2]))
            else:
                results.append(None)
        return results
