# STL 遮挡配置

仿真支持通过 STL 网格模型模拟 LED 板外壳对 LED 的遮挡，使用**射线–网格求交 + 背面剔除**算法。

## 前提

```bash
pip install trimesh
```

## 配置步骤

1. 将 STL 放到 `config/meshes/` 目录下
2. 保证 STL 与 `mocap_config.json` 中的 `points_3d` **同源、同坐标系**
3. 通过以下两种方式之一指定网格

### 方式 A：写在板卡 JSON 里

在 `mocap_config.json` 对应 `boards[]` 条目中增加：

```json
"mesh_stl": "meshes/ir_array_with_grooves.stl",
"mesh_units": "mm"
```

`mesh_units` 省略时默认为 `mm`；若 STL 已是米，设为 `"m"`。

### 方式 B：命令行传入

```bash
# 单板
python sim_publisher.py --mesh config/meshes/board.stl

# 双板（分别指定）
python sim_publisher.py --mesh 0:config/meshes/left.stl --mesh 1:config/meshes/right.stl

# 同时覆盖 points_3d
python sim_publisher.py --mesh path/to/model.stl --points-3d path/to/leds_only.json
```

## 相关参数

| 参数 | 说明 |
|------|------|
| `--no-occlusion` | 关闭遮挡逻辑（即使已配置 STL） |
| `--occlusion-eps-mm` | 射线深度容差，默认 1.5mm，减轻 LED 贴面时的自命中 |
| `--no-backface-cull` | 只做射线遮挡，不做背面剔除 |

启动后终端会打印 `Occlusion: ON` / `OFF` / `no mesh`。左侧面板 `Vis: L=x/n` 显示剔除后的可见 LED 数。
