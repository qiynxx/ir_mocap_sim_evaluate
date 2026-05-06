# Mocap Simulation Platform

双目红外动捕仿真平台，生成仿真 IR 图像并通过 ZMQ 发送给 `mocap_ir_cpp`。

## 数据流

```
sim_publisher.py ──[IR图像 5552/5553]──→ mocap_main (C++) ──[位姿 5556]──→ pose_visualizer.py
       ↑                                                                            │
       └──────────────────────── 评估回路：订阅 5556，与 GT 对比 ──────────────────┘
```

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 生成仿真标定

```bash
# 从真实标定文件自动提取（推荐）
python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json

# 或手动指定参数
python generate_sim_calib.py --fx 714 --baseline 0.0735 --width 1280 --height 800
```

### 3. 启动仿真

```bash
# 单板
python sim_publisher.py

# 双板（CPU 遮挡）
python sim_publisher.py -c config/mocap_config.json --calib config/calibration_sim.json --board-count 2

# 带 STL 遮挡
python sim_publisher.py --mesh config/meshes/board.stl
```

### 4. 一键启动完整链路

推荐优先使用一键脚本，它会按顺序启动以下 3 个组件：

1. `sim_publisher.py`
2. `mocap_ir_cpp/bin/mocap_main`
3. `mocap_ir_cpp/scripts/run_visualizer.sh`

```bash
# 使用默认配置
./scripts/run_full_pipeline.sh

# 使用指定的 mocap 配置文件
./scripts/run_full_pipeline.sh /home/zm/mocap_ir/mocap_ir_all/mocap_config_runtime.json

# 等价写法
./scripts/run_full_pipeline.sh \
  --mocap-config /home/zm/mocap_ir/mocap_ir_all/mocap_config_runtime.json
```

可选参数：

- `--no-viz`：不启动可视化器
- `--sim-only`：只启动 `sim_publisher.py`
- `--calib PATH`：指定仿真标定文件

说明：

- 脚本会自动把同一个 `mocap` 配置文件传给仿真端和 C++ 管线。
- 脚本会在 `mocap_ir_cpp` 目录下启动 `mocap_main`，避免其相对路径配置文件加载失败。
- 按 `Ctrl+C` 会一并停止脚本拉起的所有进程。

### 5. 手动启动 C++ 管线

```bash
cd ../mocap_ir_cpp
./bin/mocap_main \
  --zmq-host localhost \
  --calib ../mocap_sim_platform/config/calibration_sim.json \
  --mocap-config ../mocap_sim_platform/config/mocap_config.json \
  --zmq --display
```

> C++ 管线默认连接 `192.168.100.1`（真实硬件），仿真时必须指定 `--zmq-host localhost`。

### 6. 手动启动 3D 可视化器（可选）

```bash
cd ../mocap_ir_cpp
./scripts/run_visualizer.sh
```

## 常用参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-c` / `--config` | 板卡配置 JSON | `config/mocap_config.json` |
| `--calib` | 标定文件 | `config/calibration_sim.json` |
| `--board-count` | 仿真板卡数量 | `1` |
| `--fps` | 目标帧率 | `60` |
| `--port` | ZMQ 基础端口（左相机），右相机为 port+1 | `5552` |
| `--mesh` | STL 遮挡网格，格式 `IDX:path.stl` | — |
| `--no-occlusion` | 关闭遮挡 | — |
| `--trajectory` | 轨迹回放文件 | — |

## 文件说明

| 文件 | 说明 |
|------|------|
| `sim_publisher.py` | 主入口：交互 + IR 生成 + ZMQ 发送 + 评估展示 |
| `sim_camera.py` | 仿真双目相机 |
| `sim_ir_renderer.py` | IR 灰度图生成器 |
| `sim_scene.py` | 板模型 + 噪声 + 场景管理 |
| `sim_occlusion.py` | STL 遮挡引擎（射线检测 + 背面剔除） |
| `sim_evaluator.py` | 实时位姿评估 |
| `generate_sim_calib.py` | 生成仿真标定文件 |
| `setup_sim.py` | 从设计工具导出目录一键配置 |
| `config/calibration_sim.json` | 仿真标定文件 |
| `config/mocap_config.json` | LED 板模型配置 |
| `config/mocap_config_dual.json` | 双板配置（由 `--board-count 2` 自动生成） |
| `config/meshes/` | STL 遮挡网格 |
| `config/trajectories/` | 轨迹回放文件 |
| `logs/` | 轨迹回放误差 CSV 日志 |

## 详细文档

| 文档 | 内容 |
|------|------|
| [doc/camera_calibration.md](doc/camera_calibration.md) | 相机参数对齐、fx/FOV/基线说明 |
| [doc/occlusion.md](doc/occlusion.md) | STL 遮挡配置详细说明 |
| [doc/ui_controls.md](doc/ui_controls.md) | 界面布局、鼠标/键盘操控、评估指标、轨迹格式 |
