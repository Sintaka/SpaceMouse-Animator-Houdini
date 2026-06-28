# SpaceMouse for Houdini

3Dconnexion SpaceExplorer driver for Houdini -- camera navigation + object manipulation.

---

## English

### Overview

Reads raw HID data from a 3Dconnexion SpaceExplorer (VID:046D PID:C627) at ~125 Hz, broadcasts via UDP to a Houdini Python Panel receiver. The receiver toggles between two modes:

- **Camera mode** (default): 3DxWare official driver handles viewport navigation natively.
- **Object mode**: driver is muted via XML config; SpaceMouse moves selected objects through Channel List scoped parameters, `setWorldTransform()`, or APEX state internals.

### Supported node types

| Type | Method |
|------|--------|
| OBJ (cam, geo, null, etc.) | `setWorldTransform()` |
| Transform SOP / Edit SOP | `parm.isScoped()` -> regex match -> `parm.set()` |
| Rig Pose | `parm.isScoped()` -> regex match multiparm instances (`t0x`, `r1y`, etc.) |
| APEX Scene Animate | `state.control_selection` + `rig.graph_parms` |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` direct write |

### Files

```
src/
  spacemouse_sender.py          # HID reader + UDP broadcaster
  spacemouse_receiver_houdini.py # Houdini Python Panel receiver
  protocol.txt                   # HID report format reference
HID_SpaceExplorer.txt            # Device HID descriptor dump
```

### Setup

1. Install `hidapi` Python package: `pip install hidapi`
2. Run `spacemouse_sender.py` (external Python, needs HID access)
3. In Houdini, open Python Panel editor, load `spacemouse_receiver_houdini.py`
4. Click **Start** to begin receiving

### Usage

- **Camera mode**: just use SpaceMouse normally. Official 3DxWare driver handles everything.
- **Object mode**: click the Driver toggle button (green -> orange). Select a node with a viewport handle, the SpaceMouse now moves the object instead.
- **Per-axis gain**: 6 spinboxes allow per-axis inversion and scaling (-1.0 to 1.0).
- **Detect Parms**: prints currently scoped/handled parameters to the console.

### Axis mapping (SpaceExplorer)

| Raw | Direction | Houdini mapping |
|-----|-----------|-----------------|
| Tx | left(-) / right(+) | X (inverted) |
| Ty | fwd(-) / back(+) | Z (inverted) |
| Tz | down(+) / up(-) | Y (inverted) |
| Rx | Pitch fwd(-)/back(+) | X rotation (inverted) |
| Ry | Roll cw(-)/ccw(+) | Z rotation (inverted) |
| Rz | Yaw cw(-)/ccw(+) | Y rotation |

### Driver mute mechanism

The receiver edits `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml`, toggling `<Enabled>` on all 6 axes. 3DxWare monitors the file and applies changes without restart.

---

## 中文

### 概述

从 3Dconnexion SpaceExplorer 读取原始 HID 数据 (~125 Hz)，通过 UDP 广播到 Houdini Python Panel 接收端。接收端在两种模式间切换：

- **相机模式**（默认）：3DxWare 官方驱动原生控制视口导航
- **物体模式**：通过 XML 配置静音驱动，SpaceMouse 通过 Channel List 参数、`setWorldTransform()` 或 APEX 状态接口操控选中物体

### 支持的节点类型

| 类型 | 实现方式 |
|------|---------|
| OBJ (cam, geo, null 等) | `setWorldTransform()` |
| Transform SOP / Edit SOP | `parm.isScoped()` -> 正则匹配 -> `parm.set()` |
| Rig Pose | `parm.isScoped()` -> 正则匹配 multiparm 实例 (`t0x`, `r1y` 等) |
| APEX Scene Animate | `state.control_selection` + `rig.graph_parms` |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` 直写 |

### 文件

```
src/
  spacemouse_sender.py          # HID 读取 + UDP 广播
  spacemouse_receiver_houdini.py # Houdini Python Panel 接收端
  protocol.txt                   # HID Report 格式参考
HID_SpaceExplorer.txt            # 设备 HID 描述符
```

### 安装

1. 安装 `hidapi`：`pip install hidapi`
2. 运行 `spacemouse_sender.py`（外部 Python，需要 HID 访问权限）
3. Houdini 中打开 Python Panel 编辑器，加载 `spacemouse_receiver_houdini.py`
4. 点击 **Start** 开始接收数据

### 使用

- **相机模式**：正常使用 SpaceMouse，官方驱动处理一切
- **物体模式**：点击驱动切换按钮（绿色 -> 橙色），选中带视口 handle 的节点，SpaceMouse 即可操控物体
- **逐轴增益**：6 个数值框可独立反相和缩放每个轴 (-1.0 ~ 1.0)
- **检测参数**：打印当前 scope / handle 中的参数名到控制台

### 轴映射 (SpaceExplorer)

| 原始值 | 方向 | Houdini 映射 |
|--------|------|-------------|
| Tx | 左(-) / 右(+) | X（取反） |
| Ty | 前(-) / 後(+) | Z（取反） |
| Tz | 下(+) / 上(-) | Y（取反） |
| Rx | Pitch 前倾(-)/後仰(+) | X 旋转（取反） |
| Ry | Roll 顺时针(-)/逆时针(+) | Z 旋转（取反） |
| Rz | Yaw 顺时针(-)/逆时针(+) | Y 旋转 |

### 驱动静音机制

接收端编辑 `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml`，翻转 6 个轴的 `<Enabled>` 开关。3DxWare 实时监控该文件，无需重启即可生效。
