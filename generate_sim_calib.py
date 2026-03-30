"""
Generate a Kalibr-format calibration JSON for the simulation camera.

Produces config/calibration_sim.json with ideal pinhole cameras (zero distortion)
that match the simulation parameters. The C++ pipeline (mocap_ir_cpp) reads this
file via --calib to correctly rectify and triangulate simulated images.

Usage:
    python generate_sim_calib.py                          # defaults matching real OV9281 hardware
    python generate_sim_calib.py --fov 80 --baseline 0.12
    python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json
"""

import argparse
import json
import math
import os
from datetime import datetime

import numpy as np

# Defaults derived from real OV9281 stereo pair in calibration_full.json
_DEFAULT_WIDTH = 1280
_DEFAULT_HEIGHT = 800
_DEFAULT_FX = 717.0
_DEFAULT_FY = 717.0
_DEFAULT_CX = 633.6   # average of cam2 (629.68) and cam3 (637.58)
_DEFAULT_CY = 393.8   # average of cam2 (379.51) and cam3 (408.14)
_DEFAULT_BASELINE = 0.0735  # meters, from real T_BS


def _extract_real_params(calib_path,
                         left_key="cam2_ov9281_0",
                         right_key="cam3_ov9281_1"):
    """Read intrinsics and baseline from a real Kalibr calibration JSON."""
    with open(calib_path, "r") as f:
        calib = json.load(f)

    cameras = calib["cameras"]
    left = cameras[left_key]
    right = cameras[right_key]

    width, height = left["resolution"]
    fx_l, fy_l, cx_l, cy_l = left["intrinsics"]
    fx_r, fy_r, cx_r, cy_r = right["intrinsics"]

    # Average intrinsics
    fx = (fx_l + fx_r) / 2.0
    fy = (fy_l + fy_r) / 2.0
    cx = (cx_l + cx_r) / 2.0
    cy = (cy_l + cy_r) / 2.0

    # Compute baseline from T_BS
    T_left = np.array(left["T_BS"]["data"], dtype=np.float64).reshape(4, 4)
    T_right = np.array(right["T_BS"]["data"], dtype=np.float64).reshape(4, 4)
    T_right_left = T_right @ np.linalg.inv(T_left)
    baseline = float(np.linalg.norm(T_right_left[:3, 3]))

    fov = math.degrees(2.0 * math.atan(width / (2.0 * fx)))

    print(f"[from-calib] Loaded from {calib_path}")
    print(f"  Resolution : {width} x {height}")
    print(f"  Avg fx/fy  : {fx:.2f} / {fy:.2f}")
    print(f"  Avg cx/cy  : {cx:.2f} / {cy:.2f}")
    print(f"  Baseline   : {baseline*100:.2f} cm")
    print(f"  Equiv. FOV : {fov:.1f}°")

    return width, height, fx, fy, cx, cy, baseline, fov


def generate_calibration(width, height, baseline,
                         fx=None, fy=None, cx=None, cy=None,
                         fov_deg=None):
    """Build a Kalibr-style calibration dict for an ideal stereo pair.

    The two cameras are placed symmetrically about the body-frame origin,
    separated along the camera X axis (left-right in the sensor frame).
    """
    # Resolve intrinsics: explicit fx/fy take priority, else derive from FOV
    if fx is None:
        if fov_deg is None:
            fov_deg = math.degrees(2.0 * math.atan(width / (2.0 * _DEFAULT_FX)))
        fov_rad = math.radians(fov_deg)
        fx = (width / 2.0) / math.tan(fov_rad / 2.0)
    if fy is None:
        fy = fx
    if cx is None:
        cx = width / 2.0
    if cy is None:
        cy = height / 2.0
    if fov_deg is None:
        fov_deg = math.degrees(2.0 * math.atan(width / (2.0 * fx)))

    half_b = baseline / 2.0

    # Symmetric placement: body origin at midpoint between the two cameras.
    # cam_pos_body = -t_BS  (for R_BS = I)
    # Left camera at body X = -half_b  →  t_BS = +half_b
    # Right camera at body X = +half_b →  t_BS = -half_b
    T_BS_left = [
        1, 0, 0, half_b,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    ]
    T_BS_right = [
        1, 0, 0, -half_b,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    ]

    def make_cam(comment, T_BS_data, intrinsics):
        return {
            "sensor_type": "camera",
            "comment": comment,
            "T_BS": {
                "cols": 4,
                "rows": 4,
                "data": T_BS_data,
            },
            "rate_hz": 30,
            "resolution": [width, height],
            "camera_model": "pinhole",
            "intrinsics": intrinsics,
            "distortion_model": "radial-tangential",
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0],
            "calibration_resolution": [width, height],
        }

    calib = {
        "calibration_date": datetime.now().isoformat(),
        "sim_parameters": {
            "fov_degrees": round(fov_deg, 2),
            "baseline_m": baseline,
            "note": "Simulation calibration – ideal pinhole, zero distortion, symmetric placement",
        },
        "cameras": {
            "cam2_ov9281_0": make_cam(
                "sim left camera",
                T_BS_left,
                [fx, fy, cx, cy],
            ),
            "cam3_ov9281_1": make_cam(
                "sim right camera",
                T_BS_right,
                [fx, fy, cx, cy],
            ),
        },
    }
    return calib


def main():
    parser = argparse.ArgumentParser(
        description="Generate simulation calibration file (Kalibr format)"
    )
    parser.add_argument("--width", type=int, default=None,
                        help=f"Image width (default: {_DEFAULT_WIDTH})")
    parser.add_argument("--height", type=int, default=None,
                        help=f"Image height (default: {_DEFAULT_HEIGHT})")
    parser.add_argument("--fov", type=float, default=None,
                        help="Horizontal FOV in degrees (overrides --fx)")
    parser.add_argument("--fx", type=float, default=None,
                        help=f"Focal length fx in pixels (default: {_DEFAULT_FX})")
    parser.add_argument("--fy", type=float, default=None,
                        help="Focal length fy in pixels (default: same as fx)")
    parser.add_argument("--cx", type=float, default=None,
                        help="Principal point cx (default: width/2)")
    parser.add_argument("--cy", type=float, default=None,
                        help="Principal point cy (default: height/2)")
    parser.add_argument("--baseline", type=float, default=None,
                        help=f"Stereo baseline in meters (default: {_DEFAULT_BASELINE})")
    parser.add_argument("--from-calib", type=str, default=None,
                        help="Read intrinsics and baseline from a real calibration JSON")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output path (default: config/calibration_sim.json)")
    args = parser.parse_args()

    # Resolve parameters: --from-calib > explicit args > defaults
    if args.from_calib:
        width, height, fx, fy, cx, cy, baseline, _ = _extract_real_params(args.from_calib)
        # CLI overrides still take priority
        width = args.width if args.width is not None else width
        height = args.height if args.height is not None else height
        fx = args.fx if args.fx is not None else fx
        fy = args.fy if args.fy is not None else fy
        cx = args.cx if args.cx is not None else cx
        cy = args.cy if args.cy is not None else cy
        baseline = args.baseline if args.baseline is not None else baseline
        fov_deg = args.fov
    else:
        width = args.width if args.width is not None else _DEFAULT_WIDTH
        height = args.height if args.height is not None else _DEFAULT_HEIGHT
        fx = args.fx if args.fx is not None else _DEFAULT_FX
        fy = args.fy if args.fy is not None else _DEFAULT_FY
        cx = args.cx if args.cx is not None else _DEFAULT_CX
        cy = args.cy if args.cy is not None else _DEFAULT_CY
        baseline = args.baseline if args.baseline is not None else _DEFAULT_BASELINE
        fov_deg = args.fov  # None means "derive from fx"

    if fov_deg is not None:
        fx = None  # FOV takes priority
        fy = None

    calib = generate_calibration(width, height, baseline,
                                 fx=fx, fy=fy, cx=cx, cy=cy,
                                 fov_deg=fov_deg)

    if args.output is None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "calibration_sim.json")
    else:
        out_path = args.output

    with open(out_path, "w") as f:
        json.dump(calib, f, indent=2)

    cam = calib["cameras"]["cam2_ov9281_0"]
    fx_out = cam["intrinsics"][0]
    fov_out = calib["sim_parameters"]["fov_degrees"]
    bl_out = calib["sim_parameters"]["baseline_m"]
    print(f"\nCalibration written to: {out_path}")
    print(f"  Resolution : {width} x {height}")
    print(f"  FOV        : {fov_out}°")
    print(f"  Focal len  : fx={cam['intrinsics'][0]:.2f}  fy={cam['intrinsics'][1]:.2f}")
    print(f"  Principal  : cx={cam['intrinsics'][2]:.2f}  cy={cam['intrinsics'][3]:.2f}")
    print(f"  Baseline   : {bl_out * 100:.2f} cm  (symmetric ±{bl_out/2*100:.2f} cm)")
    print(f"  Distortion : zero (radial-tangential)")
    print(f"  Camera keys: cam2_ov9281_0 / cam3_ov9281_1")


if __name__ == "__main__":
    main()
