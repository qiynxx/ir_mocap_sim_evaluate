# 仿真相机参数对齐

仿真平台生成的 IR 图像必须与 C++ 管线使用**相同的相机参数**（焦距、基线、分辨率），否则三角化出的 3D 点位置会有系统性偏差。

## 核心参数说明

| 参数 | 含义 | 对 LED 成像的影响 |
|------|------|------------------|
| **fx / fy** (px) | 焦距，决定透视投影强度 | fx 越大 → FOV 越窄 → 远近 LED 间距差异越小 |
| **cx / cy** (px) | 光心，图像中心偏移 | 影响 LED 在图像中的位置偏移 |
| **baseline** (m) | 双目基线距离 | 基线越大 → 视差越大 → 三角化精度越高 |
| **width × height** | 图像分辨率 | 必须和 C++ 端一致 |
| **FOV** (°) | 水平视场角，与 fx 互推：`FOV = 2 × arctan(width / (2 × fx))` | FOV 越大 → 可见范围越广 |

## 对齐方式

### 方式一：自动从真实标定提取（推荐）

```bash
python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json
```

### 方式二：手动指定 fx + baseline

```bash
python generate_sim_calib.py --fx 714 --baseline 0.0735 --width 1280 --height 800
# 也可指定 cx/cy（默认取 width/2 和 height/2）
python generate_sim_calib.py --fx 714 --baseline 0.0735 --cx 629 --cy 387
```

### 方式三：通过 FOV 指定

```bash
python generate_sim_calib.py --fov 83.5 --baseline 0.0735
python generate_sim_calib.py --fov 104  --baseline 0.065
```

## 验证参数

```bash
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

## 硬件配置参考

| 参数 | OV9281 双目 (仿真默认) | IR Stereo Camera |
|------|----------------------|-----------------|
| 分辨率 | 1280×800 | 1920×1080 |
| 基线 | 73.5 mm | 65.5 mm |
| 焦距 fx | ~714 px | ~745 px |
| FOV | ~83.5° | ~104° |
| **对齐命令** | `python generate_sim_calib.py --from-calib ../mocap_ir_cpp/config/calibration_full.json` | `python generate_sim_calib.py --fx 745 --baseline 0.065 --width 1920 --height 1080` |

> 仿真使用零畸变模型。真实相机的鱼眼/径向畸变由 C++ 管线的 `stereoRectify` 在接收图像后去除，仿真跳过这一步。

## fx 与透视效果的关系

LED 板在图像中的像素间距：`像素间距 = fx × 物理间距 / 距离`

以 LED 间距 30mm 为例：

| fx (px) | FOV (°) | 0.3m 时像素间距 | 0.5m | 1.0m | 2.0m |
|---------|---------|----------------|------|------|------|
| 500 | 103° | 50 px | 30 px | 15 px | 7.5 px |
| 717 | 83° | 71.7 px | 43.0 px | 21.5 px | 10.8 px |
| 1109 | 60° | 110.9 px | 66.5 px | 33.3 px | 16.6 px |
