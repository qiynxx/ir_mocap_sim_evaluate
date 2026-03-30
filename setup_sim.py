#!/usr/bin/env python3
"""
一键仿真配置脚本。

从设计工具导出文件夹自动完成：
  1. 复制 STL 模型到 config/meshes/
  2. 生成 config/mocap_config.json
  3. 检查并按需生成 config/calibration_sim.json

用法:
    python setup_sim.py <导出文件夹路径>
    python setup_sim.py ../3D_model_generate_2D_array/output/controller_v2
    python setup_sim.py /absolute/path/to/export_folder

可选参数:
    --board-name    板卡显示名称（默认从文件夹名推导）
    --use-world     使用世界坐标代替局部坐标
    --no-calib      跳过标定文件检查
"""

import argparse
import glob
import json
import math
import os
import shutil
import subprocess
import sys


def _find_file(directory, pattern):
    """Find first file matching a glob pattern in directory."""
    matches = glob.glob(os.path.join(directory, pattern))
    return matches[0] if matches else None


def _list_stl(directory):
    """List all .stl files in directory."""
    if not os.path.isdir(directory):
        return []
    return [
        os.path.join(directory, f)
        for f in sorted(os.listdir(directory))
        if f.lower().endswith(".stl")
    ]


def _compute_initial_yaw(points_3d, origin_pos, stl_path):
    """Compute the yaw angle that orients the LED surface toward -X (cameras)."""
    try:
        import trimesh
        import numpy as np
    except ImportError:
        print("  [auto-yaw] trimesh 不可用, 使用默认 yaw=90°")
        return 90.0

    if stl_path is None or not os.path.isfile(stl_path):
        print("  [auto-yaw] STL 不存在, 使用默认 yaw=90°")
        return 90.0

    mesh = trimesh.load(stl_path, force="mesh")
    origin = np.asarray(origin_pos if origin_pos else [0, 0, 0], dtype=np.float64)
    if np.any(origin != 0):
        mesh.vertices = mesh.vertices - origin

    mesh.vertices *= 0.001
    mesh.fix_normals()

    leds = np.array(points_3d, dtype=np.float64) * 0.001
    _, _, face_ids = mesh.nearest.on_surface(leds)
    avg_n = mesh.face_normals[face_ids].mean(axis=0)
    n_len = np.linalg.norm(avg_n)
    if n_len < 1e-9:
        return 90.0
    avg_n /= n_len

    best_yaw = 0.0
    best_score = -2.0
    for yaw_deg in range(0, 360, 5):
        rad = math.radians(yaw_deg)
        c, s = math.cos(rad), math.sin(rad)
        rx = c * avg_n[0] - s * avg_n[1]
        score = -rx
        if score > best_score:
            best_score = score
            best_yaw = float(yaw_deg)

    print(f"  [auto-yaw] 平均 LED 法线: {avg_n.round(3).tolist()} → 最优 yaw={best_yaw:.0f}°")
    return best_yaw


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, "config")
    meshes_dir = os.path.join(config_dir, "meshes")

    parser = argparse.ArgumentParser(
        description="一键配置仿真：从设计工具导出文件夹自动完成所有仿真配置"
    )
    parser.add_argument(
        "export_dir",
        help="设计工具导出的文件夹路径（如 ../3D_model_generate_2D_array/output/controller_v2）"
    )
    parser.add_argument(
        "--board-name", default=None,
        help="板卡显示名称（默认从文件夹名推导）"
    )
    parser.add_argument(
        "--use-world", action="store_true",
        help="使用世界坐标代替局部坐标"
    )
    parser.add_argument(
        "--no-calib", action="store_true",
        help="跳过标定文件检查/生成"
    )
    args = parser.parse_args()

    export_dir = os.path.abspath(args.export_dir)
    if not os.path.isdir(export_dir):
        print(f"错误: 导出目录不存在: {export_dir}")
        sys.exit(1)

    folder_name = os.path.basename(export_dir)
    board_name = args.board_name or folder_name
    print(f"{'='*60}")
    print(f"  仿真配置: {folder_name}")
    print(f"{'='*60}")
    print(f"  导出目录: {export_dir}")
    print(f"  板卡名称: {board_name}")
    print()

    # ── Step 1: 复制 STL ──────────────────────────────────────────
    print("[1/3] 复制 STL 模型文件...")
    mesh_dir_src = os.path.join(export_dir, "meshes")
    stl_files = _list_stl(mesh_dir_src)

    if not stl_files:
        print(f"  警告: 在 {mesh_dir_src} 中未找到 STL 文件")
        print("  仿真将以无遮挡模式运行")
        mesh_stl_rel = None
        stl_target = None
    else:
        os.makedirs(meshes_dir, exist_ok=True)
        stl_target = None
        copied_files = []
        for src in stl_files:
            fname = os.path.basename(src)
            dst = os.path.join(meshes_dir, fname)
            shutil.copy2(src, dst)
            copied_files.append(fname)
            if stl_target is None:
                stl_target = dst
                mesh_stl_rel = f"meshes/{fname}"
            print(f"  复制: {fname}")
        print(f"  共复制 {len(copied_files)} 个 STL 文件到 {meshes_dir}")

    # ── Step 2: 生成 mocap_config.json ────────────────────────────
    print("\n[2/3] 生成 mocap_config.json...")
    coord_dir = os.path.join(export_dir, "coordinates")
    if args.use_world:
        coord_file = _find_file(coord_dir, "*_world_coordinates.json")
        coord_key = "world_position"
    else:
        coord_file = _find_file(coord_dir, "*_local_coordinates.json")
        coord_key = "local_position"

    if not coord_file:
        print(f"  错误: 在 {coord_dir} 中找不到坐标 JSON 文件")
        sys.exit(1)

    print(f"  读取坐标: {coord_file}")
    with open(coord_file, "r", encoding="utf-8") as f:
        coord_data = json.load(f)

    points = coord_data.get("points", [])
    if not points:
        print("  错误: 坐标文件中没有 points 数据")
        sys.exit(1)

    points_3d = []
    point_ids = []
    for p in points:
        pos = p.get(coord_key)
        if pos is None:
            pos = p.get("world_position", [0, 0, 0])
        points_3d.append(pos)
        point_ids.append(p["id"])

    print(f"  提取到 {len(points_3d)} 个 LED 点")
    for i, p in enumerate(points):
        name = p.get("name", f"P{i}")
        pos = points_3d[i]
        flags = []
        if p.get("is_center"):
            flags.append("center")
        if p.get("is_origin"):
            flags.append("origin")
        flag_str = f" ({', '.join(flags)})" if flags else ""
        print(f"    {name}: [{pos[0]:+8.3f}, {pos[1]:+8.3f}, {pos[2]:+8.3f}] mm{flag_str}")

    origin_pos = coord_data.get("origin_position", [0, 0, 0])
    axis_rot = coord_data.get("axis_rotation", [0, 0, 0])

    if origin_pos and any(v != 0 for v in origin_pos):
        print(f"  网格原点偏移 (mm): {origin_pos}")

    initial_yaw = _compute_initial_yaw(points_3d, origin_pos, stl_target)

    board_config = {
        "name": board_name,
        "points_3d": points_3d,
        "point_ids": point_ids,
        "mesh_stl": mesh_stl_rel,
        "mesh_units": "mm",
        "mesh_origin_world_mm": origin_pos if origin_pos else [0, 0, 0],
        "mesh_axis_rotation": axis_rot if axis_rot else [0, 0, 0],
        "initial_yaw_deg": initial_yaw,
        "cylinder_radius": 0,
        "width": 100,
        "height": 70,
    }

    config = {"boards": [board_config]}

    config_path = os.path.join(config_dir, "mocap_config.json")
    os.makedirs(config_dir, exist_ok=True)
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=4, ensure_ascii=False)

    print(f"  已生成: {config_path}")

    # ── Step 3: 检查标定文件 ──────────────────────────────────────
    calib_path = os.path.join(config_dir, "calibration_sim.json")
    if args.no_calib:
        print("\n[3/3] 跳过标定文件检查 (--no-calib)")
    elif os.path.isfile(calib_path):
        print(f"\n[3/3] 标定文件已存在: {calib_path}")
    else:
        print(f"\n[3/3] 标定文件不存在，自动生成...")
        gen_script = os.path.join(script_dir, "generate_sim_calib.py")
        if os.path.isfile(gen_script):
            ret = subprocess.run(
                [sys.executable, gen_script, "-o", calib_path],
                cwd=script_dir
            )
            if ret.returncode == 0:
                print(f"  已生成: {calib_path}")
            else:
                print(f"  警告: 标定文件生成失败 (exit code {ret.returncode})")
                print(f"  请手动运行: python generate_sim_calib.py")
        else:
            print(f"  警告: 找不到 {gen_script}")
            print(f"  请手动运行: python generate_sim_calib.py")

    # ── 完成 ──────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  配置完成!")
    print(f"{'='*60}")
    print(f"  板卡: {board_name}")
    print(f"  LED:  {len(points_3d)} 个点")
    if mesh_stl_rel:
        print(f"  STL:  {mesh_stl_rel}")
    print()
    print(f"启动仿真:")
    print(f"  cd {script_dir}")
    print(f"  python sim_publisher.py")
    print()
    print(f"连接 C++ 管线 (可选):")
    print(f"  cd ../mocap_ir_cpp/build")
    print(f"  ./mocap_main --source zmq \\")
    print(f"      --zmq-left tcp://localhost:5552 --zmq-right tcp://localhost:5553 \\")
    print(f"      --calib ../../mocap_sim_platform/config/calibration_sim.json \\")
    print(f"      --mocap-config ../../mocap_sim_platform/config/mocap_config.json \\")
    print(f"      --display")


if __name__ == "__main__":
    main()
