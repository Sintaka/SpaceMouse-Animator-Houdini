# spacemouse_receiver.pypanel
"""
SpaceMouse 接收端 — 第一人称视口相机控制
==========================================
适配活动视口: 锁定到相机节点则修改节点, No Cam 则直接操作视口 persp 相机

设备: 3Dconnexion SpaceExplorer (VID:046D PID:C627)
轴映射:
  Tx: 左(-)/右(+) → 沿相机 X 侧移      Rx: Pitch 前倾(-)/後仰(+)
  Ty: 前(-)/後(+) → 沿相机 Z 前後      Ry: Roll  顺时针(-)/逆时针(+)
  Tz: 下(+)/上(-) → 沿世界 Y 升降      Rz: Yaw   顺时针(-)/逆时针(+)

Houdini 矩阵 (row-major): v_world = v_local * M
  平移在 Row 3 | Row 0=right Row 1=up Row 2=fwd
"""
import hou
import socket
import json
import os
import xml.etree.ElementTree as ET
import numpy as np
from PySide6 import QtCore, QtWidgets

UDP_PORT = 9876

# 3DxWare 驱动配置路径
DRIVER_CFG = os.path.join(
    os.environ.get('APPDATA', ''),
    r'3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml')


# ═══════════════════════════════════════════════════════════════
# 矩阵工具
# ═══════════════════════════════════════════════════════════════

def mat4_to_numpy(m):
    """hou.Matrix4 → numpy 4x4 (float64)"""
    return np.array([
        [m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3)],
        [m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3)],
        [m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3)],
        [m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3)],
    ], dtype=np.float64)


def numpy_to_mat4(pos, rot):
    """pos(3,) + rot(3,3) → hou.Matrix4"""
    m = hou.Matrix4()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
        m.setAt(i, 3, 0.0)
    m.setAt(3, 0, float(pos[0]))
    m.setAt(3, 1, float(pos[1]))
    m.setAt(3, 2, float(pos[2]))
    m.setAt(3, 3, 1.0)
    return m


def numpy_to_mat3(rot):
    """numpy 3x3 → hou.Matrix3"""
    m = hou.Matrix3()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
    return m


def rodrigues(axis, angle_rad):
    """绕任意轴旋转的 3x3 矩阵 (column-vector convention)"""
    axis = axis / np.linalg.norm(axis)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    t = 1.0 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,     t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z,   t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y,   t*y*z + s*x, t*z*z + c],
    ], dtype=np.float64)


def orthonormalize(rot):
    """轻量正交化 — 保留 forward (row2)"""
    fwd = rot[2, :].copy()
    fwd /= np.linalg.norm(fwd)
    right = rot[0, :].copy()
    right -= np.dot(right, fwd) * fwd
    right /= np.linalg.norm(right)
    up = np.cross(fwd, right)
    up /= np.linalg.norm(up)
    out = rot.copy()
    out[0, :] = right
    out[1, :] = up
    out[2, :] = fwd
    return out


# ═══════════════════════════════════════════════════════════════
# 主面板
# ═══════════════════════════════════════════════════════════════

class SpaceMouseReceiver(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.setblocking(False)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_spacemouse)

        self.active = False
        self.frame_count = 0

        self.init_ui()

    # ── UI ────────────────────────────────────────────────

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # ── 位姿 / 相机状态 ──
        self.status_label = QtWidgets.QLabel("等待 SpaceMouse 数据...")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(110)
        self.status_label.setStyleSheet(
            "QLabel { font-family: Consolas, monospace; font-size: 12px;"
            " background: #1e1e1e; color: #d4d4d4; padding: 8px;"
            " border: 1px solid #444; border-radius: 4px; }")
        layout.addWidget(self.status_label)

        # ── 按钮 ──
        btn_row = QtWidgets.QHBoxLayout()

        self.toggle_btn = QtWidgets.QPushButton("▶  启动")
        self.toggle_btn.clicked.connect(self.toggle)
        self.toggle_btn.setMinimumHeight(30)
        btn_row.addWidget(self.toggle_btn)

        self.pose_btn = QtWidgets.QPushButton("📷 打印位姿")
        self.pose_btn.clicked.connect(self.print_pose)
        self.pose_btn.setMinimumHeight(30)
        btn_row.addWidget(self.pose_btn)

        layout.addLayout(btn_row)

        # ── 驱动开关 ──
        drv_row = QtWidgets.QHBoxLayout()
        drv_row.addWidget(QtWidgets.QLabel("3DxWare 驱动:"))

        self.driver_btn = QtWidgets.QPushButton("🎥  相机模式")
        self.driver_btn.clicked.connect(self.toggle_driver)
        self.driver_btn.setMinimumHeight(30)
        self.driver_btn.setToolTip(
            "切换 3DxWare 驱动的 Houdini 轴开关\n"
            "相机模式: 驱动控制视口, 我们不动\n"
            "物体模式: 驱动静音, 我们控制选中物体")
        drv_row.addWidget(self.driver_btn)

        # 初始状态: 驱动已禁用 (XML 中所有 Enabled=false)
        self.driver_enabled = False
        self._update_driver_btn()

        layout.addLayout(drv_row)

        # ── 灵敏度 (T/R 一行) ──
        sens = QtWidgets.QGroupBox("灵敏度 (raw / 350 × 幅值)")
        sens_row = QtWidgets.QHBoxLayout()

        sens_row.addWidget(QtWidgets.QLabel("T:"))
        self.t_spin = QtWidgets.QDoubleSpinBox()
        self.t_spin.setRange(0.0001, 10.0)
        self.t_spin.setValue(0.05)
        self.t_spin.setSingleStep(0.005)
        self.t_spin.setDecimals(5)
        sens_row.addWidget(self.t_spin)

        sens_row.addSpacing(12)

        sens_row.addWidget(QtWidgets.QLabel("R°:"))
        self.r_spin = QtWidgets.QDoubleSpinBox()
        self.r_spin.setRange(0.0001, 10.0)
        self.r_spin.setValue(1.0)
        self.r_spin.setSingleStep(0.1)
        self.r_spin.setDecimals(5)
        sens_row.addWidget(self.r_spin)

        sens_row.addStretch()
        sens.setLayout(sens_row)
        layout.addWidget(sens)

        # ── 逐轴增益 ──
        gain = QtWidgets.QGroupBox("逐轴增益 (±1.0, 负值=翻转)")
        gain_lay = QtWidgets.QGridLayout()

        t_labels = ["Tx (左右)", "Ty (前後)", "Tz (上下)"]
        r_labels = ["Rx (Pitch)", "Ry (Roll)", "Rz (Yaw)"]
        t_defaults = [1.0, 1.0, 1.0]
        r_defaults = [1.0, 1.0, -1.0]

        self.gain_t = []
        self.gain_r = []

        for col in range(3):
            lb = QtWidgets.QLabel(t_labels[col])
            lb.setStyleSheet("QLabel { font-size: 9pt; }")
            gain_lay.addWidget(lb, 0, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(t_defaults[col])
            sp.setSingleStep(0.1)
            sp.setDecimals(3)
            gain_lay.addWidget(sp, 1, col)
            self.gain_t.append(sp)

        for col in range(3):
            lb = QtWidgets.QLabel(r_labels[col])
            lb.setStyleSheet("QLabel { font-size: 9pt; }")
            gain_lay.addWidget(lb, 2, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(r_defaults[col])
            sp.setSingleStep(0.1)
            sp.setDecimals(3)
            gain_lay.addWidget(sp, 3, col)
            self.gain_r.append(sp)

        gain.setLayout(gain_lay)
        layout.addWidget(gain)

        self.setLayout(layout)

    # ── 启动/停止 ────────────────────────────────────────

    def toggle(self):
        if self.active:
            self.timer.stop()
            self.toggle_btn.setText("▶  启动")
            self.active = False
            self.status_label.setText("已停止")
        else:
            self.timer.start(4)
            self.toggle_btn.setText("⏸  停止")
            self.active = True
            self.status_label.setText("已启动，等待数据...")

    # ── 驱动开关 ─────────────────────────────────────────

    def toggle_driver(self):
        """切换 3DxWare 驱动的 Houdini 轴启用/禁用"""
        new_state = not self.driver_enabled
        if self._write_driver_enabled(new_state):
            self.driver_enabled = new_state
            self._update_driver_btn()

    def _update_driver_btn(self):
        if self.driver_enabled:
            self.driver_btn.setText("🎥  相机模式 (驱动开)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #2a622a; color: #fff; }")
        else:
            self.driver_btn.setText("🧰  物体模式 (驱动关)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #6a4a1a; color: #fff; }")

    def _write_driver_enabled(self, enable):
        """修改 SideFX_HoudiniFX.xml 中所有轴的 <Enabled> 值, 保存后驱动实时加载"""
        try:
            if not os.path.exists(DRIVER_CFG):
                self.status_label.setText(f"找不到驱动配置:\n{DRIVER_CFG}")
                return False

            tree = ET.parse(DRIVER_CFG)
            root = tree.getroot()

            changed = 0
            for axis in root.iter('Axis'):
                en = axis.find('Enabled')
                if en is not None:
                    new_val = 'true' if enable else 'false'
                    if en.text != new_val:
                        en.text = new_val
                        changed += 1

            if changed > 0:
                tree.write(DRIVER_CFG, encoding='UTF-8', xml_declaration=True)
                self.status_label.setText(
                    f"驱动: {'启用' if enable else '禁用'} {changed} 个轴 → 已写入\n"
                    f"3DxWare 实时监控 XML, 约 1 秒内生效")
            else:
                self.status_label.setText(
                    f"驱动: 轴已处于 {'启用' if enable else '禁用'} 状态 (未改动)")

            return True

        except Exception as e:
            self.status_label.setText(f"驱动配置写入失败: {e}")
            return False

    # ── 打印位姿 ─────────────────────────────────────────

    def print_pose(self):
        """打印当前视口相机位姿到控制台"""
        try:
            viewport = self._get_viewport()
            if viewport is None:
                print("错误: 未找到活动视口")
                return

            m = mat4_to_numpy(viewport.viewTransform())
            pos = m[3, :3]
            rot = m[:3, :3]
            view_dir = -rot[2, :]

            info = (
                f"=== 视口相机位姿 ===\n"
                f"相机   : {self._camera_label(viewport)}\n"
                f"位置   : ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})\n"
                f"视野   : ({view_dir[0]:+.4f}, {view_dir[1]:+.4f}, {view_dir[2]:+.4f})\n"
                f"Right  : ({rot[0,0]:+.4f}, {rot[0,1]:+.4f}, {rot[0,2]:+.4f})\n"
                f"Up     : ({rot[1,0]:+.4f}, {rot[1,1]:+.4f}, {rot[1,2]:+.4f})"
            )
            print(info)
            self.status_label.setText(info)

        except Exception as e:
            self.status_label.setText(f"打印错误: {e}")

    # ── 视口辅助 ─────────────────────────────────────────

    def _get_viewport(self):
        """获取当前活动 SceneViewer 的 curViewport, 失败返回 None"""
        try:
            desktop = hou.ui.curDesktop()
            viewer = desktop.paneTabOfType(hou.paneTabType.SceneViewer)
            if viewer is None:
                return None
            return viewer.curViewport()
        except Exception:
            return None

    def _camera_label(self, viewport):
        """返回视口当前相机的人类可读标签"""
        cam_node = viewport.camera()
        if cam_node is not None:
            name = cam_node.path()
            if 'spacemouse_viewport_cam' in name:
                return f"🔧 {name} (persp代理)"
            return f"🎯 {name} (节点)"
        else:
            return "🔄 No Cam (persp)"

    # ── UDP 接收 + 移动 ──────────────────────────────────

    def update_from_spacemouse(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            pkt = json.loads(data.decode('utf-8'))

            tx, ty, tz = pkt['translation']
            rx, ry, rz = pkt['rotation']

            t_sens = self.t_spin.value()
            r_sens = self.r_spin.value()

            gt = np.array([g.value() for g in self.gain_t], dtype=np.float64)
            gr = np.array([g.value() for g in self.gain_r], dtype=np.float64)

            tv = np.array([tx, ty, tz], dtype=np.float64) / 350.0 * t_sens * gt
            rv = np.array([rx, ry, rz], dtype=np.float64) / 350.0 * r_sens * gr

            self.move_viewport_camera(tv[0], tv[1], tv[2], rv[0], rv[1], rv[2])
            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)

        except BlockingIOError:
            pass
        except Exception as e:
            self.status_label.setText(f"错误: {e}")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        viewport = self._get_viewport()
        cam_label = self._camera_label(viewport) if viewport else "???"

        m = mat4_to_numpy(viewport.viewTransform()) if viewport else np.eye(4)
        pos = m[3, :3]

        self.status_label.setText(
            f"{cam_label}\n"
            f"in  T:({tx:+4d},{ty:+4d},{tz:+4d})  R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
            f"out T:({tv[0]:+.5f},{tv[1]:+.5f},{tv[2]:+.5f})  "
            f"R:({rv[0]:+.4f}°,{rv[1]:+.4f}°,{rv[2]:+.4f}°)\n"
            f"pos ({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})")

    # ── 核心: 视口自适应相机控制 ──────────────────────────

    def move_viewport_camera(self, tx, ty, tz, rx, ry, rz):
        """第一人称视口相机控制

        统一路径: 全部走相机节点 setWorldTransform() 原子写回

        GeometryViewportCamera (pivot/translation/rotation) 无法原子更新:
          setPivot/setTranslation/setRotation 各自触发视口刷新,
          三元组中间态不一致 → 每帧闪屏.
          setDefaultCamera() 内部先 reset 到默认 persp → 同样闪屏.

        结论: Houdini Python API 只有 setWorldTransform(node) 支持无闪烁视口更新.
              因此 No Cam 时用代理相机节点接管, 之后全部走此路径.
        """
        viewport = self._get_viewport()
        if viewport is None:
            return

        # 1. 从 viewTransform() 读世界位姿 (权威来源, 不依赖任何节点)
        m = mat4_to_numpy(viewport.viewTransform())
        world_pos = m[3, :3].copy()
        rot = m[:3, :3].copy()

        # 2. 局部轴
        cam_right = rot[0, :]
        cam_fwd   = rot[2, :]
        world_up  = np.array([0.0, 1.0, 0.0])

        # 3. 第一人称平移增量
        world_delta = tx * cam_right + ty * cam_fwd
        world_delta[1] -= tz
        world_pos += world_delta

        # 4. 第一人称旋转 — Yaw → Pitch → Roll
        if abs(rz) > 1e-10:
            R = rodrigues(world_up, np.radians(rz))
            rot = rot @ R.T

        if abs(rx) > 1e-10:
            R = rodrigues(rot[0, :], np.radians(rx))
            rot = rot @ R.T

        if abs(ry) > 1e-10:
            R = rodrigues(rot[2, :], np.radians(ry))
            rot = rot @ R.T

        rot = orthonormalize(rot)

        # 5. 获取或创建代理相机节点 (有节点直接用, 无节点创建一个)
        cam_node = self._get_or_create_proxy_cam(viewport)

        # 6. 单次原子写入 — setWorldTransform 是唯一无闪烁路径
        cam_node.setWorldTransform(numpy_to_mat4(world_pos, rot))

    def _get_or_create_proxy_cam(self, viewport):
        """获取视口当前相机; No Cam 时创建透明代理节点并接管视口"""
        cam_node = viewport.camera()

        if cam_node is not None:
            return cam_node  # 已有相机, 直接用

        # No Cam → 创建或复用代理相机
        proxy_path = '/obj/spacemouse_viewport_cam'
        proxy = hou.node(proxy_path)
        if proxy is None:
            proxy = hou.node('/obj').createNode('cam', 'spacemouse_viewport_cam')
            proxy.setGenericFlag(hou.nodeFlag.Display, False)     # 视口不可见
            proxy.setGenericFlag(hou.nodeFlag.Selectable, False)  # 不可选中
            # 标记为模板/参考, 使其在层级中低调显示
            try:
                proxy.setGenericFlag(hou.nodeFlag.Template, True)
            except Exception:
                pass

        # 将当前视口自由视角烤入代理相机 → 无缝接管
        viewport.saveViewToCamera(proxy)
        viewport.setCamera(proxy)       # 视口切到代理相机

        return proxy


def createInterface():
    return SpaceMouseReceiver()
