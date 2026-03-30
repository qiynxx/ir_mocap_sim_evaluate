# 从手柄设计到仿真验证：完整工作流程

本文档描述如何将 `3D_model_generate_2D_array`（设计工具）中完成的手柄设计导入 `mocap_sim_platform`（仿真平台），并运行仿真验证。

## 整体流程概览

```
┌──────────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│  1. 设计工具完成设计    │     │  2. 一键配置仿真    │     │  3. 启动仿真       │
│  点击「导出」按钮      │ ──→ │  setup_sim.py     │ ──→ │  sim_publisher    │
│  自动导出到 output/   │     │  自动完成所有配置   │     │                  │
└──────────────────────┘     └──────────────────┘     └──────────────────┘
```

**所需时间**：设计完成后，约 30 秒即可完成导入并启动仿真。

---

## 前置条件

```bash
cd mocap_sim_platform
pip install -r requirements.txt
```

确保已安装 `trimesh`、`numpy`、`opencv-python`、`pygame`、`pyopengl`、`pyzmq` 等依赖。

---

## 第一步：在设计工具中完成设计并导出

在 `3D_model_generate_2D_array` 工具中完成手柄 IR LED 阵列的设计后，点击「一键保存所有文件到项目文件夹」按钮。

**导出行为**：
- 默认导出路径为 `3D_model_generate_2D_array/output/`
- 文件夹名称**自动设为加载的 STL 文件名**（去掉扩展名），例如加载 `controller_v2.stl` 后导出到 `output/controller_v2/`
- 原始 STL 始终导出，无论是否成功生成凹槽

导出目录结构示例：

```
3D_model_generate_2D_array/output/
├── controller_v2/                    # 自动命名 = STL 文件名
│   ├── meshes/
│   │   ├── controller_v2_original.stl          # 原始 3D 曲面模型（始终导出）
│   │   └── controller_v2_with_grooves.stl      # 带凹槽模型（如果有）
│   ├── coordinates/
│   │   ├── controller_v2_local_coordinates.json # LED 局部坐标（仿真使用此文件）
│   │   └── controller_v2_world_coordinates.json # LED 世界坐标
│   ├── drawings/
│   │   ├── controller_v2_fpc_layout.svg         # FPC 布局图
│   │   └── ...
│   ├── config/
│   │   └── controller_v2_config.json            # 设计参数
│   └── README.txt
│
├── gamepad_prototype/                # 另一个设计方案
│   ├── meshes/
│   │   └── ...
│   └── ...
```

**关键文件**：
| 文件 | 用途 |
|------|------|
| `meshes/*_original.stl` | 手柄 3D 模型，用于遮挡检测和 3D 可视化 |
| `coordinates/*_local_coordinates.json` | LED 点在局部坐标系下的位置，包含坐标原点和旋转信息 |

---

## 第二步：一键配置仿真

`setup_sim.py` 脚本自动完成所有仿真配置，只需提供导出文件夹路径：

```bash
cd mocap_sim_platform
python setup_sim.py ../3D_model_generate_2D_array/output/controller_v2
```

脚本自动完成以下操作：
1. **复制 STL** → `config/meshes/`
2. **生成 `config/mocap_config.json`** — 板卡配置（含 LED 坐标、STL 路径、最优朝向角度）
3. **检查 `config/calibration_sim.json`** — 不存在则自动生成虚拟双目标定

### 输出示例

```
============================================================
  仿真配置: controller_v2
============================================================
  导出目录: /path/to/3D_model_generate_2D_array/output/controller_v2
  板卡名称: controller_v2

[1/3] 复制 STL 模型文件...
  复制: controller_v2_original.stl
  共复制 1 个 STL 文件到 config/meshes

[2/3] 生成 mocap_config.json...
  读取坐标: .../controller_v2_local_coordinates.json
  提取到 5 个 LED 点
    P1: [ +28.779,  -14.520,   -3.612] mm
    P2: [ +11.835,   -8.045,   -8.491] mm
    P3: [  +0.000,   +0.000,   +0.000] mm (origin)
    P4: [ -12.905,   -1.779,   -4.746] mm
    P5: [ -22.605,   +1.234,   +3.886] mm
  [auto-yaw] 平均 LED 法线: [0.164, 0.863, -0.479] → 最优 yaw=90°
  已生成: config/mocap_config.json

[3/3] 标定文件已存在: config/calibration_sim.json

============================================================
  配置完成!
============================================================
```

### 可选参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `export_dir` | 导出文件夹路径（必填） | - |
| `--board-name` | 板卡显示名称 | 文件夹名 |
| `--use-world` | 使用世界坐标 | 否 |
| `--no-calib` | 跳过标定文件检查 | 否 |

---

## 第三步：启动仿真

```bash
python sim_publisher.py
```

### 仿真界面说明

启动后会显示一个窗口，包含：

| 区域 | 内容 |
|------|------|
| 中央 3D 视图 | 上帝视角，显示手柄 3D 模型和 LED 点（红色=可见，灰色=被遮挡） |
| 左下角 LEFT CAM | 左相机 IR 图像预览，蓝色圆圈标注检测到的 LED |
| 右下角 RIGHT CAM | 右相机 IR 图像预览，蓝色圆圈标注检测到的 LED |
| 左侧面板 | 控制说明、噪声状态、相机参数、板卡信息 |
| 右上角 | 评估面板（连接 C++ 管线后显示精度数据） |

### 交互控制

| 按键 | 功能 |
|------|------|
| `1` / `2` | 切换选中的板卡 |
| `W` / `S` | 前 / 后移动（X 轴） |
| `A` / `D` | 左 / 右移动（Y 轴） |
| `Q` / `E` | 上 / 下移动（Z 轴） |
| `R` / `F` | Roll 旋转（绕 X） |
| `T` / `G` | Pitch 旋转（绕 Y） |
| `Y` / `H` | Yaw 旋转（绕 Z） |
| 鼠标拖拽 | 旋转上帝视角 |
| 滚轮 | 缩放 |
| `N` | 开关噪声点 |
| `V` | 开关评估面板 |

### 遮挡效果

旋转手柄时可以观察到：
- LED 可见数量随角度变化（如 `Vis: L=3/5 R=4/5`）
- 背对相机的 LED 在 IR 图像中消失
- 被手柄几何体遮挡的 LED 同样不可见

---

## 第四步（可选）：连接 C++ 管线进行精度评估

仿真平台通过 ZMQ 发送 IR 图像，可以连接 `mocap_ir_cpp` 管线进行端到端精度评估：

```bash
# 终端 1：仿真发布器（已启动）
python sim_publisher.py

# 终端 2：C++ 检测管线
cd ../mocap_ir_cpp
./bin/mocap_main \
  --zmq-host localhost \
  --calib ../mocap_sim_platform/config/calibration_sim.json \
  --mocap-config ../mocap_sim_platform/config/mocap_config.json \
  --zmq --display
```

> **注意**：C++ 管线有自己独立的 `config/` 目录（`mocap_ir_cpp/config/`），`setup_sim.py` 只修改 `mocap_sim_platform/config/` 下的文件，不会影响 C++ 端的任何配置。

连接后，仿真窗口右上角的 EVALUATION 面板会显示实时精度指标（位置误差、旋转误差）。

---

## 常见问题

### Q: 启动仿真后相机看不到 LED？

`setup_sim.py` 会自动计算 `initial_yaw_deg`（初始 yaw 角），使 LED 表面在启动时面向相机。如果仍看不到，尝试使用 `Y` / `H` 键旋转板卡。

### Q: 仿真帧率很低？

3D 模型面数过多（>100K 面）可能导致遮挡计算变慢。可以在建模软件中简化网格，或使用以下命令跳过遮挡：

```bash
python sim_publisher.py --no-occlusion
```

### Q: 如何更换手柄设计？

重复第一步和第二步：在设计工具中修改设计 → 重新导出 → 运行 `setup_sim.py` → 重启仿真。

### Q: 导出目录在哪里？

默认位置：`3D_model_generate_2D_array/output/<STL文件名>/`。在设计工具中加载 STL 后，项目名称自动设为 STL 文件名。

---

## 目录结构参考

```
mocap_ir_all/
├── 3D_model_generate_2D_array/          # 设计工具
│   └── output/                          # 默认导出根目录
│       ├── controller_v2/               # 导出数据（文件夹名=STL文件名）
│       │   ├── meshes/                  # STL 模型
│       │   ├── coordinates/             # LED 坐标
│       │   ├── drawings/                # FPC 图纸
│       │   └── config/                  # 设计参数
│       └── gamepad_prototype/           # 另一个设计
│
├── mocap_sim_platform/                  # 仿真平台
│   ├── setup_sim.py                     # 一键配置脚本（第二步使用）
│   ├── config/
│   │   ├── mocap_config.json            # 板卡配置（setup_sim.py 生成）
│   │   ├── calibration_sim.json         # 相机标定（自动检查/生成）
│   │   └── meshes/                      # STL 模型（setup_sim.py 复制）
│   ├── sim_publisher.py                 # 仿真主程序（第三步启动）
│   ├── convert_export_to_sim.py         # 转换脚本（高级用法，可独立使用）
│   ├── generate_sim_calib.py            # 标定生成（高级用法，可独立使用）
│   ├── sim_occlusion.py                 # 遮挡引擎
│   ├── sim_camera.py                    # 虚拟双目相机
│   ├── sim_scene.py                     # 场景管理
│   ├── sim_ir_renderer.py               # IR 图像生成
│   └── docs/
│       ├── design_to_simulation_guide.md    # 本文档
│       └── occlusion_sim_design.md          # 遮挡仿真技术方案
│
└── mocap_ir_cpp/                        # C++ 检测管线（可选，独立配置）
    └── config/                          # C++ 自有配置（不受 setup_sim.py 影响）
```

---

## 快速命令汇总

```bash
# 从导出到启动仿真，只需 2 条命令：
cd mocap_sim_platform

# 1. 一键配置（复制 STL + 生成配置 + 检查标定）
python setup_sim.py ../3D_model_generate_2D_array/output/controller_v2

# 2. 启动仿真
python sim_publisher.py
```
