# 界面说明与操控

## 窗口布局

```
┌──────────────────────────────────────────────────────────────────┐
│  MOCAP SIM PUBLISHER  │  Frames: 1234  │  Cam: 1280x800  │  60 FPS │  ← 顶栏
├─────────────┬──────────────────────────────────────┬─────────────┤
│ 控制说明     │                                      │ 评估面板     │
│ 板信息       │         3D 上帝视角 (OpenGL)          │ (按 V 切换)  │
│ 相机信息     │                                      │             │
├─────────────┴──────────────┬─────────────────────────┴─────────────┤
│   LEFT CAM IR 预览          │   RIGHT CAM IR 预览                    │
└─────────────────────────────┴────────────────────────────────────────┘
```

## 鼠标操控

| 操作 | 功能 |
|------|------|
| 左键拖拽 | 旋转上帝视角 |
| 滚轮 | 缩放视角 |
| 右键拖拽 | 平移板卡 XY |
| Shift + 右键拖拽 | 旋转板卡 Pitch / Yaw |
| Ctrl + 右键拖拽 | 旋转板卡 Roll |
| Ctrl + 滚轮 | 升降板卡 Z 轴 |

## 键盘操控

| 按键 | 功能 |
|------|------|
| F1 - F9 | 跳转预设位姿 |
| P | 开始/暂停轨迹回放 |
| Tab / 1 / 2 | 切换选中板 |
| N | 切换噪声 |
| V | 切换评估面板 |
| W/S | 精细前后移动 (X) |
| A/D | 精细左右移动 (Y) |
| Q/E | 精细上下移动 (Z) |
| R/F | 精细 Roll 旋转 |
| T/G | 精细 Pitch 旋转 |
| Y/H | 精细 Yaw 旋转 |

## 预设位姿（F1-F9）

| 按键 | 名称 | 说明 |
|------|------|------|
| F1 | Front Close | 0.4m，正对相机 |
| F2 | Front Mid | 1.0m，正对相机 |
| F3 | Front Far | 2.5m，正对相机 |
| F4 | Left 30° | 1.0m，左偏 + Yaw 30° |
| F5 | Right 30° | 1.0m，右偏 + Yaw -30° |
| F6 | Above 45° | 0.8m，高处 + Pitch 45° |
| F7 | Oblique | 1.2m，复合偏移 + 旋转 |
| F8 | Extreme Angle | 0.6m，大角度 Yaw 60° |
| F9 | Far Tilted | 3.0m，远距 + 微倾 |

## 评估面板指标

按 `V` 切换显示，订阅 C++ 管线位姿输出（ZMQ port 5556），与仿真 GT 实时对比。

| 指标 | 含义 | 颜色阈值 |
|------|------|---------|
| **Pos err** (mm) | 位置误差欧氏距离 | 绿 <5mm, 黄 <15mm, 红 ≥15mm |
| **dX / dY / dZ** (mm) | 三轴位置误差分量 | — |
| **Rot err** (°) | 旋转矩阵角度差 | 绿 <2°, 黄 <5°, 红 ≥5° |
| **dR / dP / dY** (°) | Roll/Pitch/Yaw 误差分量 | — |
| **Avg** (mm) | 最近 100 帧位置误差均值 | — |
| **Max** (mm) | 最近 100 帧位置误差峰值 | — |
| **RMSE** (mm) | C++ 端 PnP 重投影误差（C++ 回传） | — |
| **Inliers** | PnP 内点数（LED 匹配数） | — |

## 轨迹回放

按 `P` 启动，自动遍历位姿并将误差记录到 `logs/traj_*.csv`。

```bash
# 使用内置预设轨迹
python sim_publisher.py
# 启动后按 P

# 加载自定义轨迹
python sim_publisher.py --trajectory config/trajectories/distance_sweep.json
```

### 自定义轨迹格式

```json
{
    "name": "my_test",
    "loop": false,
    "dwell_frames": 60,
    "interpolate": true,
    "waypoints": [
        {"position": [x, y, z], "euler": [roll, pitch, yaw]},
        ...
    ]
}
```

| 字段 | 说明 |
|------|------|
| `dwell_frames` | 每个路径点停留帧数 |
| `loop` | 是否循环播放 |
| `interpolate` | 是否平滑插值 |
| `position` | 位置 [X, Y, Z]（米，FLU） |
| `euler` | 旋转 [Roll, Pitch, Yaw]（度） |

### 内置轨迹文件

| 文件 | 说明 |
|------|------|
| `config/trajectories/distance_sweep.json` | 距离扫描：0.3m → 3.0m |
| `config/trajectories/angle_sweep.json` | 角度扫描：Yaw ±60° 往返 |
| `config/trajectories/full_test.json` | 综合测试 |
