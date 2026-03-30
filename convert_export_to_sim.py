#!/usr/bin/env python3
"""
将 3D_model_generate_2D_array 导出的数据转换为 mocap_sim_platform 可用的配置。

从 ir_array_export/ 目录读取局部坐标 JSON，生成：
  1. mocap_config.json — 板卡配置（含 points_3d、point_ids、mesh_stl 路径）
  2. 打印 STL 文件应放置的目标路径

用法:
    python convert_export_to_sim.py                              # 使用默认路径
    python convert_export_to_sim.py --export-dir /path/to/ir_array_export
    python convert_export_to_sim.py --board-name "My Board"
    python convert_export_to_sim.py --stl-filename my_model.stl  # STL 文件名
"""

import argparse
import json
import math
import os
import sys


def _compute_initial_yaw(points_3d, origin_pos, axis_rot, mesh_stl_rel,
                         stl_target_dir):
    """Compute the yaw angle (degrees) that orients the LED surface toward -X.

    Uses trimesh (if available and STL exists) to find the average LED surface
    normal, then picks the yaw that maximises dot(rotated_normal, [-1,0,0]).
    Falls back to 90° if trimesh is unavailable.
    """
    try:
        import trimesh
        import numpy as np
    except ImportError:
        print("  [auto-yaw] trimesh 不可用, 使用默认 yaw=90°")
        return 90.0

    stl_path = None
    if mesh_stl_rel:
        candidate = os.path.join(stl_target_dir, os.path.basename(mesh_stl_rel))
        if os.path.isfile(candidate):
            stl_path = candidate
    if stl_path is None:
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
        score = -rx  # dot with [-1,0,0]
        if score > best_score:
            best_score = score
            best_yaw = float(yaw_deg)

    print(f"  [auto-yaw] 平均 LED 法线: {avg_n.round(3).tolist()} → 最优 yaw={best_yaw:.0f}°")
    return best_yaw


def main():
    default_export_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "3D_model_generate_2D_array", "output", "ir_array_export"
    )
    default_output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "config", "mocap_config.json"
    )

    parser = argparse.ArgumentParser(
        description="将设计工具导出转换为仿真平台配置"
    )
    parser.add_argument(
        "--export-dir", default=default_export_dir,
        help="ir_array_export 目录路径"
    )
    parser.add_argument(
        "-o", "--output", default=default_output,
        help="输出 mocap_config.json 路径"
    )
    parser.add_argument(
        "--board-name", default="IR Array Export",
        help="板卡名称"
    )
    parser.add_argument(
        "--stl-filename", default=None,
        help="STL 文件名（放在 config/meshes/ 下），省略则自动检测 meshes/ 目录"
    )
    parser.add_argument(
        "--use-world", action="store_true",
        help="使用世界坐标而非局部坐标（不推荐，除非未设置原点）"
    )
    args = parser.parse_args()

    export_dir = os.path.abspath(args.export_dir)
    if not os.path.isdir(export_dir):
        print(f"错误: 导出目录不存在: {export_dir}")
        sys.exit(1)

    # --- 读取坐标 ---
    coord_dir = os.path.join(export_dir, "coordinates")
    if args.use_world:
        coord_file = _find_file(coord_dir, "*_world_coordinates.json")
        coord_key = "world_position"
    else:
        coord_file = _find_file(coord_dir, "*_local_coordinates.json")
        coord_key = "local_position"

    if not coord_file:
        print(f"错误: 在 {coord_dir} 中找不到坐标 JSON 文件")
        sys.exit(1)

    print(f"读取坐标: {coord_file}")
    with open(coord_file, "r", encoding="utf-8") as f:
        coord_data = json.load(f)

    points = coord_data.get("points", [])
    if not points:
        print("错误: 坐标文件中没有 points 数据")
        sys.exit(1)

    points_3d = []
    point_ids = []
    for p in points:
        pos = p.get(coord_key)
        if pos is None:
            pos = p.get("world_position", [0, 0, 0])
        points_3d.append(pos)
        point_ids.append(p["id"])

    print(f"提取到 {len(points_3d)} 个 LED 点")
    for i, p in enumerate(points):
        name = p.get("name", f"P{i}")
        pos = points_3d[i]
        flags = []
        if p.get("is_center"):
            flags.append("center")
        if p.get("is_origin"):
            flags.append("origin")
        flag_str = f" ({', '.join(flags)})" if flags else ""
        print(f"  {name}: [{pos[0]:+8.3f}, {pos[1]:+8.3f}, {pos[2]:+8.3f}] mm{flag_str}")

    # --- 检测/配置 STL ---
    mesh_stl_rel = None
    stl_target_dir = os.path.join(os.path.dirname(args.output), "meshes")

    if args.stl_filename:
        mesh_stl_rel = f"meshes/{args.stl_filename}"
        stl_target = os.path.join(stl_target_dir, args.stl_filename)
    else:
        mesh_dir = os.path.join(export_dir, "meshes")
        stl_files = _list_stl(mesh_dir)
        if stl_files:
            src = stl_files[0]
            fname = os.path.basename(src)
            mesh_stl_rel = f"meshes/{fname}"
            stl_target = os.path.join(stl_target_dir, fname)
            print(f"检测到导出 STL: {src}")
        else:
            stl_target = os.path.join(stl_target_dir, "board.stl")
            mesh_stl_rel = "meshes/board.stl"
            print("未检测到导出 STL，使用默认名称 board.stl")

    # --- 提取 mesh 坐标变换参数（世界→局部） ---
    origin_pos = coord_data.get("origin_position", [0, 0, 0])
    axis_rot = coord_data.get("axis_rotation", [0, 0, 0])

    if origin_pos and any(v != 0 for v in origin_pos):
        print(f"网格原点偏移 (mm): {origin_pos}")
    if axis_rot and any(v != 0 for v in axis_rot):
        print(f"网格轴旋转 (deg): {axis_rot}")

    # --- 计算 LED 面朝向并确定最优初始 yaw（使 LED 面向 -X 方向/相机） ---
    initial_yaw = _compute_initial_yaw(
        points_3d, origin_pos, axis_rot, mesh_stl_rel, stl_target_dir)

    # --- 生成 mocap_config.json ---
    board_config = {
        "name": args.board_name,
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

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=4, ensure_ascii=False)

    print(f"\n已生成: {args.output}")
    print(f"  板卡名称: {args.board_name}")
    print(f"  LED 点数: {len(points_3d)}")
    print(f"  mesh_stl: {mesh_stl_rel}")

    # --- 打印 STL 放置说明 ---
    os.makedirs(stl_target_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"请将原始 STL 文件复制到以下位置:")
    print(f"  {os.path.abspath(stl_target)}")
    print(f"{'='*60}")

    stl_exists = os.path.isfile(os.path.abspath(stl_target))
    if stl_exists:
        print("(STL 文件已存在)")
    else:
        print("(STL 文件尚未放置)")

    print(f"\n启动仿真:")
    print(f"  cd {os.path.dirname(args.output)}/..")
    print(f"  python sim_publisher.py")
    if not stl_exists:
        print(f"\n  或不带遮挡运行:")
        print(f"  python sim_publisher.py --no-occlusion")


def _find_file(directory, pattern):
    """Find first file matching a glob-like pattern in directory."""
    import glob
    matches = glob.glob(os.path.join(directory, pattern))
    return matches[0] if matches else None


def _list_stl(directory):
    """List all .stl files in directory."""
    if not os.path.isdir(directory):
        return []
    return [
        os.path.join(directory, f)
        for f in os.listdir(directory)
        if f.lower().endswith(".stl")
    ]


if __name__ == "__main__":
    main()
