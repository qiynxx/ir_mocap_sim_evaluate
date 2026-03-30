# Mocap Simulation Platform

独立的双目红外动捕仿真平台，通过 ZMQ 向 `mocap_ir_cpp` 发送仿真 IR 图像数据。

## 架构 & 数据流

仿真器作为"虚拟相机板"，替代真实硬件（OV9281 双目 + IR LED 板），生成仿真 IR 灰度图并通过 ZMQ PUB 发送。

完整数据流需要三个进程协同工作：

```
sim_publisher.py ──[IR图像 5552/5553]──→ mocap_main (C++) ──[位姿JSON 5556]──→ pose_visualizer.py
       ↑                                                          │
       └──────────────── [评估: 订阅位姿 5556] ───────────────────┘
```

```
mocap_sim_platform               mocap_ir_cpp
┌────────────────┐  ZMQ images   ┌────────────────┐  ZMQ poses   ┌──────────────┐
│ 场景 + IR 生成  │ ────────────→ │ 检测/匹配/三角  │ ───────────→ │ 3D 可视化器   │
│ port 5552 (L)  │               │ 化/姿态估计     │              │ port 5556    │
│ port 5553 (R)  │               │ --display 窗口  │              │              │
└───────┬────────┘               └────────────────┘              └──────────────┘
        │ 评估回路：订阅 5556，与 GT 对比                              │
        └─────────────────────────────────────────────────────────────┘
```

仿真器同时订阅 C++ 管线的位姿输出（端口 5556），将估计值与仿真 GT 实时对比，在界面右侧显示逐板误差指标。

## 快速开始

### 方式一：一键启动（推荐）

```bash
./scripts/run_full_pipeline.sh            # 启动全部三个组件
./scripts/run_full_pipeline.sh --no-viz   # 不启动 3D 可视化器
./scripts/run_full_pipeline.sh --sim-only # 只启动仿真发布器
```

### 方式二：从设计工具导入（推荐）

在 `3D_model_generate_2D_array` 中完成设计并导出后，使用 `setup_sim.py` 一键完成配置：

```bash
pip install -r requirements.txt

# 一键配置：自动复制 STL + 生成 mocap_config.json + 检查标定文件
python setup_sim.py ../3D_model_generate_2D_array/output/controller_v2

# 启动仿真
python sim_publisher.py
```

详细流程见 [设计到仿真指南](docs/design_to_simulation_guide.md)。

### 方式三：手动分步启动

```bash
# 终端 1: 安装依赖 + 生成标定 + 启动仿真
pip install -r requirements.txt
python generate_sim_calib.py
python sim_publisher.py
# 可选：带外壳遮挡（STL 与设计端 points_3d 同系）
# python sim_publisher.py --mesh config/meshes/your_board.stl

# 终端 2: 启动 C++ 管线（注意关键参数）
cd ../mocap_ir_cpp
./bin/mocap_main \
  --zmq-host localhost \
  --calib ../mocap_sim_platform/config/calibration_sim.json \
  --mocap-config ../mocap_sim_platform/config/mocap_config.json \
  --zmq --display

# 终端 3: 启动 3D 可视化器（可选）
cd ../mocap_ir_cpp
./scripts/run_visualizer.sh
```

> **注意**：C++ 管线默认连接 `192.168.100.1`（真实硬件），仿真时必须指定 `--zmq-host localhost`。

## 仿真相机参数对齐（重要）

仿真平台生成的 IR 图像必须与 C++ 管线使用**相同的相机参数**（焦距、基线、分辨率），否则三角化出的 3D 点位置会有系统性偏差。

### 核心参数说明

| 参数 | 含义 | 对 LED 成像的影响 |
|------|------|------------------|
| **fx / fy** (px) | 焦距，决定透视投影强度 | fx 越大 → FOV 越窄 → 远近 LED 间距差异越小（"更平"） |
| **cx / cy** (px) | 光心，图像中心偏移 | 影响 LED 在图像中的位置偏移 |
| **baseline** (m) | 双目基线距离 | 基线越大 → 视差越大 → 三角化精度越高 |
| **width × height** | 图像分辨率 | 必须和 C++ 端一致 |
| **FOV** (°) | 水平视场角，与 fx 互推：`FOV = 2 × arctan(width / (2 × fx))` | FOV 越大 → 可见范围越广，但边缘畸变（仿真不模拟畸变） |

**关键关系**：`fx` 和 `FOV` 二选一指定，互相唯一确定（给定 width）。

### 如何从真实标定文件提取参数

C++ 管线的真实标定文件 `calibration_full.json` 包含 4 个相机，仿真只关注双目 IR 相机（`cam2_ov9281_0` 和 `cam3_ov9281_1`）：

```
calibration_full.json 中的关键字段
─────────────────────────────────
cameras:
  cam2_ov9281_0:                          ← 左相机
    resolution: [1280, 800]               ← 分辨率
    intrinsics: [713.71, 713.71,          ← [fx, fy, cx, cy]
                 642.57, 378.16]
    distortion_model: "equidistant"       ← 鱼眼畸变（仿真中忽略）
    T_BS.data: [R|t 4x4]                 ← body→sensor 外参

  cam3_ov9281_1:                          ← 右相机
    resolution: [1280, 800]
    intrinsics: [714.63, 714.59,
                 615.92, 395.51]
    T_BS.data: [R|t 4x4]

基线 = ‖T_BS_right × inv(T_BS_left) 的平移部分‖ ≈ 73.5 mm
```

### 对齐步骤（三种方式）

#### 方式一：自动从真实标定提取（推荐）

```bash
python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json
```

自动提取 cam2/cam3 的 fx、fy、cx、cy、baseline，生成对应的仿真标定。输出会打印提取到的参数，便于确认。

#### 方式二：手动指定 fx + baseline

从真实标定中读取参数，手动输入：

```bash
# 1. 查看真实标定中的参数
#    cam2: fx≈713.7, cam3: fx≈714.6 → 取均值 ~714
#    分辨率: 1280×800
#    基线: ~0.0735m

# 2. 生成仿真标定（fx + baseline）
python generate_sim_calib.py --fx 714 --baseline 0.0735 --width 1280 --height 800

# 也可以指定 cx/cy（默认取 width/2 和 height/2）
python generate_sim_calib.py --fx 714 --baseline 0.0735 --cx 629 --cy 387
```

#### 方式三：通过 FOV 指定

如果你知道目标相机的水平视场角：

```bash
python generate_sim_calib.py --fov 83.5 --baseline 0.0735    # OV9281 等效 FOV
python generate_sim_calib.py --fov 60 --baseline 0.0735      # 窄视角，LED 近大远小更缓和
python generate_sim_calib.py --fov 104 --baseline 0.065      # 广角 IR Stereo Camera
```

### 验证参数是否正确

生成后检查 `config/calibration_sim.json`：

```bash
# 查看生成的参数
python -c "
import json
with open('config/calibration_sim.json') as f:
    c = json.load(f)
cam = c['cameras']['cam2_ov9281_0']
print(f'分辨率: {cam[\"resolution\"]}')
print(f'焦距:   fx={cam[\"intrinsics\"][0]:.1f}  fy={cam[\"intrinsics\"][1]:.1f}')
print(f'光心:   cx={cam[\"intrinsics\"][2]:.1f}  cy={cam[\"intrinsics\"][3]:.1f}')
print(f'基线:   {c[\"sim_parameters\"][\"baseline_m\"]*100:.2f} cm')
print(f'FOV:    {c[\"sim_parameters\"][\"fov_degrees\"]:.1f}°')
"
```

启动仿真后，顶栏会显示 `Cam: 1280x800  FOV: 60°  BL: 7.4cm`，核对是否与预期一致。

### 透视效果与 fx 的关系

LED 板在图像中的像素间距由焦距和距离共同决定：`像素间距 = fx × 物理间距 / 距离`

以 LED 间距 30mm 为例：

| fx (px) | FOV (°) | 0.3m 时像素间距 | 0.5m | 1.0m | 2.0m |
|---------|---------|----------------|------|------|------|
| 500 | 103° | 50 px | 30 px | 15 px | 7.5 px |
| 717 | 83° | 71.7 px | 43.0 px | 21.5 px | 10.8 px |
| 1109 | 60° | 110.9 px | 66.5 px | 33.3 px | 16.6 px |

- **fx 太小**（广角）：远处 LED 挤在一起，C++ 检测可能丢点
- **fx 太大**（长焦）：近处 LED 可能超出画面
- **应对齐真实硬件**：确保仿真中看到的 LED 间距与真实相机一致

### 硬件配置参考

C++ 管线支持两种相机方案，仿真默认对齐 OV9281：

| 参数 | OV9281 双目 (仿真默认) | IR Stereo Camera |
|------|----------------------|-----------------|
| 标定文件 | `calibration_full.json` cam2/cam3 | 独立标定 |
| 分辨率 | 1280×800 | 1920×1080 |
| 连接方式 | ZMQ 分开传输 | USB side-by-side |
| 基线 | 73.5 mm | 65.5 mm |
| 焦距 fx | ~714 px | ~745 px |
| 畸变模型 | equidistant (鱼眼) | radial-tangential |
| FOV | ~83.5° | ~104° |
| **仿真对齐命令** | `python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json` | `python generate_sim_calib.py --fx 745 --baseline 0.065 --width 1920 --height 1080` |

> **注意**：仿真使用零畸变 + `radial-tangential` 模型（系数全零）。真实相机的鱼眼/径向畸变由 C++ 管线的 `stereoRectify` 在接收图像后去除，仿真跳过这一步。

### LED 板型对比

C++ 管线提供三种板型配置，仿真端可直接使用：

| 配置文件 | 板名 | LED 数 | 板数 | 点分布 | 特点 |
|---------|------|--------|------|--------|------|
| `mocap_config.json` | IR Array Right | 11 | 1 | 3D 圆柱 | 当前默认 |
| `mocap_config_pico.json` | IR Array Left + Right | 6+6 | 2 | 3D 圆柱 | 双板追踪 |
| `mocap_config_sig.json` | IR Array Left | 11 | 1 | 2D 平面 (z=0) | IMU primary 模式 |

切换板型（仿真和 C++ 必须使用同一份）：

```bash
# 仿真端
python sim_publisher.py -c ../mocap_ir_cpp/config/mocap_config_pico.json

# C++ 端（对应使用同一板型）
./bin/mocap_main --mocap-config config/mocap_config_pico.json \
  --zmq-host localhost --zmq --display
```

> **关键**：`points_3d` 坐标单位为 **毫米**，`name` 字段必须两端一致（评估模块按此匹配 GT）。

### 3D 外壳遮挡（STL，可选）

从设计工具导出带凹槽的 **STL** 后，可与 `points_3d` 使用**同一板卡坐标系**（通常为 mm），仿真会按 **射线–网格求交 + 背面剔除** 决定 LED 是否画进 IR 图；未配置网格时行为与旧版一致（仅视锥裁剪）。详细算法见 `docs/occlusion_sim_design.md`。

**依赖**：`pip install -r requirements.txt`（含 `trimesh`）。

#### 步骤概览

1. 将 STL 放到本仓库内任意路径（例如 `config/meshes/xxx_with_grooves.stl`）。
2. 保证 STL 与 `mocap_config.json` 中的 `points_3d` **同源、同坐标系**（与设计工具导出一致）。
3. 用下面**两种方式之一**指定网格后启动 `sim_publisher.py`。

#### 方式 A：写在板卡 JSON 里

在对应 `boards[]` 条目中增加（路径相对 **`mocap_config.json` 所在目录**）：

```json
"mesh_stl": "meshes/ir_array_with_grooves.stl",
"mesh_units": "mm"
```

`mesh_units` 省略时默认为 `mm`；若 STL 已是米，设为 `"m"`。

#### 方式 B：命令行传入（可覆盖 JSON）

```bash
# 仅指定 STL，默认绑定第 0 块板
python sim_publisher.py --mesh /path/to/board.stl

# 指定板索引（多块板时可重复 --mesh）
python sim_publisher.py --mesh 0:config/meshes/left.stl --mesh 1:config/meshes/right.stl

# 用独立 JSON 覆盖某块板的 points_3d（毫米），常与 --mesh 联用
python sim_publisher.py --mesh path/to/model.stl --points-3d path/to/leds_only.json --points-board 0
```

`leds_only.json` 需含顶层字段 `points_3d`（数组，与 `mocap_config` 中单板格式相同）；`point_ids` 可选。

#### 常用开关

| 参数 | 作用 |
|------|------|
| `--no-occlusion` | 关闭遮挡逻辑（即使已配置 STL） |
| `--occlusion-eps-mm` | 射线深度容差（默认 1.5 mm），减轻灯位贴在表面时的自命中 |
| `--no-backface-cull` | 只做射线遮挡，不做背面剔除 |

启动后终端会打印 `Occlusion: ON` / `OFF` / `no mesh`。左侧面板 **Vis: L=x/n** 在开启遮挡时为剔除后的可见数量。

## 界面总览

启动仿真器后，窗口布局如下：

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  MOCAP SIM PUBLISHER   │  ZMQ Frames: 1234  │  Cam: 1280x800 …  │  FPS: 30   │ ← ① 顶栏
├──────────────┬──────────────────────────────────────────────┬────────────────── │
│  CONTROLS    │                                              │ EVALUATION       │
│  1/2  选板   │                                              │ Connected (42)   │
│  WASD  移动  │                                              │                  │
│  QE   升降   │          ③ 3D 上帝视角                       │ IR Array Right   │
│  RFTGYH 旋转 │             (OpenGL)                         │ Pos err: 2.3 mm  │
│  N  噪声     │                                              │  dX=+1.1 dY=…    │
│  V  评估面板 │          · 网格 + 坐标轴                     │ Rot err: 0.85°   │
│              │          · 双目相机锥体 (L/R)                │  dR=… dP=… dY=…  │
│  NOISE       │          · LED 板模型 + LED 点               │ Avg: 3.1mm       │
│  Status: OFF │          · 噪声点 (橙色)                     │ Max: 8.2mm       │
│              │                                              │ RMSE: 1.5mm      │
│  CAMERAS     │                                              │ Inliers: 11      │
│  L [x,y,z]  │                                              │                  │
│  R [x,y,z]  │                                              │                  │
│  BL/FOV 信息 │                                              │                  │
│              │                                              │                  │
│  BOARDS      │                                              │                  │
│  > 板名      │                                              │                  │
│    Pos/Rot   │                                              │                  │
│    Dist/Vis  │                                              │                  │
├──────────────┼───────────────────────┬──────────────────────┼──────────────────┘
│ ┌──────────────────────┐             │ ┌──────────────────────┐                 │
│ │ LEFT CAM             │             │ │ RIGHT CAM            │                 │
│ │  (IR 灰度图预览)     │             │ │  (IR 灰度图预览)     │                 │
│ └──────────────────────┘             │ └──────────────────────┘                 │
│        ④ 左相机 IR                   │        ⑤ 右相机 IR                       │
└─────────────────────────────────────────────────────────────────────────────────┘

① 顶栏 ─── ZMQ 发送状态、相机参数、实时帧率
② 左面板 ── 操控说明、噪声状态、相机位置、板信息
③ 中央 ─── 3D 交互视角（鼠标拖拽旋转、滚轮缩放）
④⑤ 底部 ── 双目 IR 灰度图实时预览
⑥ 右面板 ── 评估面板（按 V 切换显示/隐藏）
```

### 各区域说明

| 区域 | 位置 | 说明 |
|------|------|------|
| 顶栏 | 顶部横条 | 显示 ZMQ 已发帧数、相机分辨率/FOV/基线、实时 FPS（绿>25, 黄>15, 红≤15）|
| 控制面板 | 左侧 | 按键帮助、噪声开关状态（ON=绿/OFF=红）|
| 相机信息 | 左侧中部 | 左右相机在世界坐标系下的位置 [X,Y,Z]、基线距离、FOV |
| 板信息 | 左侧下部 | 当前选中板（`>` 标记）的位置/旋转/到相机距离/双目可见 LED 数 |
| 3D 视角 | 中央 | FLU 世界坐标系下的上帝视角，包含网格、坐标轴（红=X 黄=Y 蓝=Z）、相机锥体、LED 板 |
| IR 预览 | 左下 / 右下 | 左右相机实时生成的 IR 灰度图，LED 以亮斑呈现，大小随距离透视缩放 |
| 评估面板 | 右侧 | 实时显示 C++ 管线估计位姿与仿真 GT 的误差（详见下文）|

### 左面板 · 板信息字段

| 字段 | 含义 |
|------|------|
| `Pos [x, y, z]` | 板中心在世界坐标系（FLU）下的位置（米） |
| `Rot [r, p, y]°` | 板的欧拉角（Roll, Pitch, Yaw，单位°） |
| `Dist` | 板中心到双目相机中点的欧氏距离（米）。绿 <3m，黄 <5m，红 ≥5m |
| `Vis: L=n/N R=n/N` | 左/右相机视野内可见的 LED 数 / 总 LED 数 |

## 评估指标详解

评估面板（按 `V` 切换）订阅 C++ 管线的位姿输出（ZMQ port 5556），将 **C++ 估计值** 与 **仿真 Ground Truth** 实时对比，逐板显示误差。

### 指标一览

| 指标 | 含义 | 颜色阈值 |
|------|------|---------|
| **Pos err** (mm) | 位置误差：估计位置与 GT 位置的欧氏距离 `‖est - gt‖ × 1000` | 绿 <5mm, 黄 <15mm, 红 ≥15mm |
| **dX / dY / dZ** (mm) | 位置误差在 X(前)/Y(左)/Z(上) 三轴上的分量，带正负号，可判断偏移方向 | — |
| **Rot err** (°) | 旋转误差：通过 `arccos((tr(R_est^T · R_gt) - 1) / 2)` 计算的旋转矩阵角度差 | 绿 <2°, 黄 <5°, 红 ≥5° |
| **dR / dP / dY** (°) | 旋转误差在 Roll/Pitch/Yaw 三轴上的分量（wrap 到 ±180°），可判断哪个轴旋转偏差大 | — |
| **Avg** (mm) | 最近 100 帧的位置误差均值，反映系统稳态精度 | — |
| **Max** (mm) | 最近 100 帧的位置误差峰值，反映最差情况 | — |
| **RMSE** (mm) | C++ 管线 PnP 求解时的重投影 RMSE（由 C++ 端计算并回传），反映 2D-3D 匹配质量 | — |
| **Inliers** | C++ 管线 PnP 求解使用的内点数（LED 匹配点数），越接近总 LED 数越好 | — |

> **理解要点**：
> - `Pos err` / `Rot err` 是 **仿真端计算** 的端到端 GT 误差，直接衡量整条管线（检测→匹配→三角化→PnP）的精度。
> - `RMSE` / `Inliers` 是 **C++ 端回传** 的内部指标，反映 PnP 求解的拟合质量和鲁棒性。
> - 当 `Inliers` 明显少于 LED 总数时，说明检测或匹配环节丢点；当 `RMSE` 较低但 `Pos err` 较高时，可能是标定参数有偏差。

## 操控说明

| 按键 | 功能 |
|------|------|
| 1 / 2 | 选择板 |
| W / S | 前后移动 (X) |
| A / D | 左右移动 (Y) |
| Q / E | 上下移动 (Z) |
| R / F | Roll 旋转 |
| T / G | Pitch 旋转 |
| Y / H | Yaw 旋转 |
| N | 切换噪声 |
| V | 切换评估面板 |
| 鼠标拖拽 | 旋转上帝视角 |
| 滚轮 | 缩放 |

## 文件说明

| 文件 | 说明 |
|------|------|
| `sim_publisher.py` | 主入口：交互 + IR 生成 + ZMQ 发送 + 评估展示 |
| `setup_sim.py` | 一键仿真配置：从设计工具导出文件夹自动完成 STL 复制 + 配置生成 + 标定检查 |
| `sim_evaluator.py` | 实时位姿评估（订阅 C++ 输出，计算 GT 误差） |
| `sim_camera.py` | 仿真双目相机（从标定 JSON 读参数） |
| `sim_ir_renderer.py` | IR 灰度图生成器（透视缩放，恒定亮度） |
| `sim_scene.py` | 板模型 + 噪声 + 场景管理；可选 `mesh_stl` / `mesh_units` |
| `sim_occlusion.py` | STL 加载、射线遮挡与背面剔除 |
| `convert_export_to_sim.py` | 转换脚本（高级用法，可独立使用） |
| `generate_sim_calib.py` | 生成仿真标定文件（支持 --from-calib） |
| `scripts/run_full_pipeline.sh` | 一键启动全链路脚本 |
| `config/calibration_sim.json` | 仿真标定文件 |
| `config/mocap_config.json` | LED 板模型配置（可选 `mesh_stl`、`mesh_units`） |
