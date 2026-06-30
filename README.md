# SpaceMouse for Houdini

3Dconnexion SpaceExplorer driver for Houdini -- camera navigation + object manipulation.
<img width="3550" height="1787" alt="屏幕截图 2026-06-29 222900" src="https://github.com/user-attachments/assets/9b56e377-3837-48cc-8dde-c7aae0a4bc4b" />
---

**⚠️ NEW: Unified version available!** `spacemouse_houdini.py` combines sender + receiver into one file. Just copy to Python Panel and run — no UDP networking needed!

**⚠️ 最新：已推出合一版本！** `spacemouse_houdini.py` 将发送端和接收端合二为一，直接复制到 Python Panel 即可运行——无需 UDP 网络转发！

**Installation requirement / 安装依赖：**
```bash
# Install hidapi in Houdini's built-in Python / 在 Houdini 内置 Python 中安装 hidapi
"C:\Program Files\Side Effects Software\Houdini 21.0.729\python311\python.exe" -m pip install hidapi

## English

### Overview

Reads raw HID data from a 3Dconnexion SpaceExplorer (VID:046D PID:C627) at ~125 Hz, broadcasts via UDP to a Houdini Python Panel receiver. The receiver toggles between two modes:

- **Camera mode** (default): 3DxWare official driver handles viewport navigation natively.
- **Object mode**: driver is muted via XML config; SpaceMouse moves selected objects through Channel List scoped parameters, `setWorldTransform()`, or APEX state internals.

### Supported node types

| Type | Method |
|------|--------|
| OBJ (cam, geo, null, etc.) | `setWorldTransform()` |
| Transform SOP | `parm.isScoped()` -> regex match -> `parm.set()` |
| Rig Pose | `parm.isScoped()` -> regex match multiparm instances (`t0x`, `r1y`, etc.) |
| APEX Scene Animate | `state.control_selection` + `rig.graph_parms` (local only, see limitations) |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` direct write |

### Files

```
src/
  spacemouse_sender.py             # HID reader + UDP broadcaster
  spacemouse_receiver_houdini.py   # Houdini Python Panel receiver
  protocol.txt                     # HID report format reference
  apex_export_control_xforms.py    # APEX Script for control xform export (experimental)
ApexScriptSamples/                 # APEX Script component examples
HID_SpaceExplorer.txt              # Device HID descriptor dump
```

### Setup

1. Install `hidapi` Python package: `pip install hidapi`
2. Run `spacemouse_sender.py` (external Python, needs HID access)
3. In Houdini, open Python Panel editor, load `spacemouse_receiver_houdini.py`
4. Click **Start** to begin receiving

### UI

```
[Start] [Detect Parms]

3DxWare: [Driver: ON (Camera)]  [Cam Space / Abs Space]

   Tx Ty Tz Rx Ry Rz  T R  All None   (right-aligned)

Sensitivity (raw/350 * gain)
  T: [0.05]  R: [1.0]
  Tx    Ty    Tz    Rx    Ry    Rz
  [1.0] [1.0] [1.0] [1.0] [1.0] [-1.0]
```

#### Controls

| Control | Function |
|---------|----------|
| **Start / Stop** | Begin/end UDP reception |
| **Detect Parms** | Print currently scoped/handled parameters to console |
| **Driver: ON/OFF** | Toggle 3DxWare axis enable/disable for Houdini |
| **Cam Space / Abs Space** | Toggle camera-relative vs world-absolute movement |
| **Tx Ty Tz Rx Ry Rz** | Per-channel on/off toggle. Alt+Click = solo. Color: X=red, Y=green, Z=blue / Gray=off |
| **T / R** | Master toggle for translation/rotation group. Green when all 3 channels on. |
| **All / None** | Enable/disable all 6 channels |

#### Per-axis gain

6 spinboxes (-1.0 to 1.0). Negative values invert the axis direction. Defaults: T=1.0, Rz=-1.0.

### Movement modes

- **Cam Space** (default): translation and rotation are camera-relative. Push forward = Z+, lift up = Y+.
- **Abs Space**: translation and rotation use world axes (X=right, Y=up, Z=forward).

APEX forces Abs Space mode (world transform not available).

### Axis mapping (SpaceExplorer)

| Raw | Direction | Houdini mapping |
|-----|-----------|-----------------|
| Tx | left(-) / right(+) | X |
| Ty | fwd(-) / back(+) | Z |
| Tz | down(+) / up(-) | Y |
| Rx | Pitch fwd(-)/back(+) | X rotation |
| Ry | Roll cw(-)/ccw(+) | Z rotation |
| Rz | Yaw cw(-)/ccw(+) | Y rotation |

### Driver mute mechanism

The receiver edits `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml`, toggling `<Enabled>` on all 6 axes. 3DxWare monitors the file and applies changes without restart.

### Known issues

- **Socket port conflict**: if you copy a new `.pypanel` file and get "port already in use", restart Houdini twice (first restart releases the old socket, second binds the new one).
- **APEX Scene Animate limitations**: APEX controls can only be moved with local transforms. World-space transform access is not available via Houdini's Python API. The viewport transform handle (`PI_Handle`) is a protected C++ class with no Python or HDK access. Hoping for improvements in Houdini 22.

### TODO

- [ ] APEX Scene Animate world-transform support (pending Houdini API updates)

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
| Transform SOP | `parm.isScoped()` -> 正则匹配 -> `parm.set()` |
| Rig Pose | `parm.isScoped()` -> 正则匹配 multiparm 实例 (`t0x`, `r1y` 等) |
| APEX Scene Animate | `state.control_selection` + `rig.graph_parms` (仅本地变换, 见限制) |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` 直写 |

### 文件

```
src/
  spacemouse_sender.py             # HID 读取 + UDP 广播
  spacemouse_receiver_houdini.py   # Houdini Python Panel 接收端
  protocol.txt                     # HID Report 格式参考
  apex_export_control_xforms.py    # APEX 脚本: 控制器世界变换导出 (实验性)
ApexScriptSamples/                 # APEX Script 组件示例
HID_SpaceExplorer.txt              # 设备 HID 描述符
```

### 安装

1. 安装 `hidapi`：`pip install hidapi`
2. 运行 `spacemouse_sender.py`（外部 Python，需要 HID 访问权限）
3. Houdini 中打开 Python Panel 编辑器，加载 `spacemouse_receiver_houdini.py`
4. 点击 **Start** 开始接收数据

### UI

```
[Start] [Detect Parms]

3DxWare: [Driver: ON (Camera)]  [Cam Space / Abs Space]

   Tx Ty Tz Rx Ry Rz  T R  All None   (右对齐)

Sensitivity (raw/350 * gain)
  T: [0.05]  R: [1.0]
  Tx    Ty    Tz    Rx    Ry    Rz
  [1.0] [1.0] [1.0] [1.0] [1.0] [-1.0]
```

#### 控件说明

| 控件 | 功能 |
|------|------|
| **Start / Stop** | 启停 UDP 接收 |
| **Detect Parms** | 打印当前 scope / handle 中的参数名到控制台 |
| **Driver: ON/OFF** | 切换 3DxWare 驱动的 Houdini 轴开关 |
| **Cam Space / Abs Space** | 切换相机相对位移 / 世界绝对位移 |
| **Tx Ty Tz Rx Ry Rz** | 单通道开关。Alt+单击 = Solo。颜色: X=红, Y=绿, Z=蓝 / 灰=关 |
| **T / R** | 平移/旋转组总开关。全开时绿色高亮 |
| **All / None** | 全部启用 / 全部禁用 |

#### 逐轴增益

6 个数值框 (-1.0 ~ 1.0)。负值翻转该轴方向。默认: T=1.0, Rz=-1.0。

### 位移模式

- **Cam Space**（默认）：平移和旋转随相机视角。前推=Z+，上提=Y+。
- **Abs Space**：平移和旋转使用世界轴 (X=右, Y=上, Z=前)。

检测到 APEX 时强制 Abs Space（无法获取世界变换）。

### 轴映射 (SpaceExplorer)

| 原始值 | 方向 | Houdini 映射 |
|--------|------|-------------|
| Tx | 左(-) / 右(+) | X |
| Ty | 前(-) / 後(+) | Z |
| Tz | 下(+) / 上(-) | Y |
| Rx | Pitch 前倾(-)/後仰(+) | X 旋转 |
| Ry | Roll 顺时针(-)/逆时针(+) | Z 旋转 |
| Rz | Yaw 顺时针(-)/逆时针(+) | Y 旋转 |

### 驱动静音机制

接收端编辑 `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml`，翻转 6 个轴的 `<Enabled>` 开关。3DxWare 实时监控该文件，无需重启即可生效。

### 已知问题

- **端口占用**：如果复制新 `.pypanel` 文件后提示端口被占用，请重启 Houdini 两次（第一次释放旧 socket，第二次绑定新的）。
- **APEX Scene Animate 限制**：APEX 控制器仅能以本地变换模式移动。Houdini Python API 无法获取控制器的世界空间变换。视口 Transform Handle (`PI_Handle`) 是 C++ 保护类方法，Python 和 HDK 均不可访问。期待 Houdini 22 改进。

### TODO

- [ ] APEX Scene Animate 世界变换支持（等待 Houdini API 更新）
