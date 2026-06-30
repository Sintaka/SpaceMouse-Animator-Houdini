# SpaceMouse for Houdini

[English](README.md)

3Dconnexion SpaceExplorer 驱动 for Houdini — 相机导航 + 物体操控。

<img width="3550" height="1787" alt="SpaceMouse Houdini Panel" src="https://github.com/user-attachments/assets/9b56e377-3837-48cc-8dde-c7aae0a4bc4b" />

---

## 安装依赖

在 Houdini 自带的 Python 中安装 `hidapi`：

```bash
"C:\Program Files\Side Effects Software\Houdini 21.0.729\python311\python.exe" -m pip install hidapi
```

## 使用方法

1. 打开 Houdini → **Windows** → **Python Panel Editor**
2. 将 [`src/panel_entry.py`](src/panel_entry.py) 的内容粘贴到新建面板中
3. **修改顶部的 `SRC_PATH`**，指向你的 `src/` 目录：

   ```python
   SRC_PATH = r"D:\code\dev\Houdini\spaceMouse1\src"
   ```

4. 点击 **Start** — 面板直接读取 SpaceMouse HID，无需外部进程。

> 旧版使用独立 sender + receiver 进程通过 UDP 通信。当前版本将所有功能整合为 `src/` 下的模块化 Python 文件。

## 文件结构

```
src/
  panel_entry.py                # 粘贴到 Houdini Python Panel 的入口脚本
  spacemouse_controller.py      # 编排器 — 连接 HID + UI + 数据处理
  ui_panel.py                   # Qt 面板 UI（纯控件，无业务逻辑）
  hid_reader.py                 # HID 设备读取（守护线程 → queue）
  data_processor.py             # 原始轴值 → 位移增量（灵敏度/增益/遮罩/重映射）
  target_detector.py            # 自动检测可移动目标（OBJ / parm / Rig Pose / APEX）
  target_mover.py               # 按目标类型应用位移
  driver_control.py             # 通过 XML 配置切换 3DxWare 驱动
  config.py                     # 硬件ID、灵敏度默认值、轴映射表
  matrix_utils.py               # numpy ↔ hou.Matrix 转换、Rodrigues
  apex_utils.py                 # APEX 骨骼查找、作用域控制检测
  debug_utils.py                # Tee-print 调试日志
  STRUCTURE.txt                 # 模块索引（每个文件一行说明）
  debug/                        # 独立测试脚本
```

## 界面

```
[Start] [Detect Parms]

3DxWare: [Driver: ON (Camera)]  [Cam Space / Abs Space]

   Tx Ty Tz Rx Ry Rz   T R   All None

Sensitivity (raw/350 * gain)
  T: [0.05]  R: [1.0]
  Tx  Ty  Tz   Rx  Ry  Rz
  [1] [1] [1]  [1] [1] [-1]
```

| 控件 | 功能 |
|------|------|
| **Start / Stop** | 启停 HID 读取 |
| **Detect Parms** | 打印当前目标参数到控制台 |
| **Driver ON / OFF** | 切换 3DxWare 轴开关 — ON=相机模式, OFF=物体模式 |
| **Cam Space / Abs Space** | 相机相对 / 世界绝对位移 |
| **Tx Ty Tz Rx Ry Rz** | 单轴开关。Alt+单击 = 独奏 |
| **T / R** | 平移 / 旋转总开关 |
| **All / None** | 全部启用 / 全部禁用 |

逐轴增益：6 个数值框 (-1.0 ~ 1.0)。负值 = 翻转方向。

## 轴映射 (SpaceExplorer)

| 原始值 | 方向 | Houdini |
|--------|------|---------|
| Tx | 左(-) / 右(+) | X |
| Ty | 前(-) / 後(+) | Z |
| Tz | 下(+) / 上(-) | Y |
| Rx | Pitch 前倾(-) / 後仰(+) | RX |
| Ry | Roll 顺时针(-) / 逆时针(+) | RZ |
| Rz | Yaw 顺时针(-) / 逆时针(+) | RY |

## 支持的目标类型

| 类型 | 实现方式 |
|------|---------|
| OBJ 节点 | `setWorldTransform()` |
| Transform SOP / Rig Pose | scoped parm → 正则匹配 → `parm.set()` |
| APEX Scene Animate | `rig.graph_parms`（仅本地变换） |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` |

## 驱动静音

面板通过修改 `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml` 中的 `<Enabled>` 来切换驱动。3DxWare 实时生效，无需重启。

## 已知问题

- **APEX**：`rig.graph_parms` 写入不可撤销（Ctrl+Z 无效），Channel List 不会自动刷新。这是 Houdini API 层面的限制（Python 无法访问 `PI_Handle`）。
- **端口冲突**（仅限旧版 UDP）：从旧 `.pypanel` 切换时如提示端口被占用，重启 Houdini 两次即可释放 socket。
