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
        self.sock.setblocking(False)
        self._sock_ok = True
        try:
            self.sock.bind(("127.0.0.1", UDP_PORT))
        except (OSError, PermissionError):
            self._sock_ok = False
            print(f"UDP {UDP_PORT} 被占用, 请切换 Panel 再切回")

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

        # ── 状态指示行 ──
        stat_row = QtWidgets.QHBoxLayout()

        # 驱动状态
        stat_row.addWidget(QtWidgets.QLabel("驱动:"))
        self.driver_status = QtWidgets.QLabel("🎥")
        self.driver_status.setToolTip("3DxWare 驱动轴状态\n绿色=驱动开(相机) 橙色=驱动关(物体)")
        stat_row.addWidget(self.driver_status)

        stat_row.addSpacing(12)

        # 可移动状态
        stat_row.addWidget(QtWidgets.QLabel("物件:"))
        self.object_status = QtWidgets.QLabel("━")
        self.object_status.setToolTip("当前选中是否可移动\n🟢=有可移动对象 ✗=无")
        stat_row.addWidget(self.object_status)

        stat_row.addStretch()

        # 切换按钮
        self.driver_btn = QtWidgets.QPushButton(" 切到物体模式 ")
        self.driver_btn.clicked.connect(self.toggle_driver)
        self.driver_btn.setMinimumHeight(26)
        self.driver_btn.setMinimumWidth(100)
        self.driver_btn.setToolTip("切换驱动轴开关\n相机模式=官方驱动控视口\n物体模式=驱动静音+选中物体可控")
        stat_row.addWidget(self.driver_btn)

        layout.addLayout(stat_row)

        # 初始: 默认相机模式 (驱动开)
        self.driver_enabled = True
        self._write_driver_enabled(True)
        self._update_driver_status()

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

        if not getattr(self, '_sock_ok', True):
            self.status_label.setText(
                f"⚠ UDP 端口 {UDP_PORT} 被占用\n"
                f"请切到另一个 Panel, 再切回来重新加载")

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
            self._update_driver_status()

    def _update_driver_status(self):
        if self.driver_enabled:
            self.driver_status.setText("🎥 相机")
            self.driver_status.setStyleSheet(
                "QLabel { color: #5f5; font-weight: bold; }")
            self.driver_btn.setText(" 切到物体模式 ")
            self.driver_btn.setStyleSheet("")
        else:
            self.driver_status.setText("🧰 物体")
            self.driver_status.setStyleSheet(
                "QLabel { color: #fa0; font-weight: bold; }")
            self.driver_btn.setText(" 切到相机模式 ")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #5a5a1a; color: #fff; }")

    def _update_object_status(self):
        """检测当前选中节点是否可移动"""
        try:
            sel = hou.selectedNodes()
            if not sel:
                self.object_status.setText("━ 无选中")
                self.object_status.setStyleSheet("QLabel { color: #888; }")
                return

            is_movable = self._get_movable_target() is not None
            node = sel[0]
            cat = node.type().category()
            name = node.name()[:14]

            if is_movable:
                if cat == hou.objNodeTypeCategory():
                    tag = "OBJ"
                elif cat == hou.sopNodeTypeCategory():
                    tag = "SOP"
                else:
                    tag = str(cat)[:3]
                self.object_status.setText(f"🟢 [{tag}] {name}")
                self.object_status.setStyleSheet(
                    "QLabel { color: #5f5; font-weight: bold; }")
            else:
                self.object_status.setText(f"✗ {name}")
                self.object_status.setStyleSheet("QLabel { color: #888; }")
        except Exception:
            self.object_status.setText("━")
            self.object_status.setStyleSheet("QLabel { color: #888; }")

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
        if not self._sock_ok:
            return
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

            if self.driver_enabled:
                # 驱动开 → 官方驱动控制相机, 我们只显示
                pass
            else:
                # 驱动关 → 只移动物体, 绝不碰相机
                target = self._get_movable_target()
                if target is not None:
                    self.move_selected_object(target, tv[0], tv[1], tv[2],
                                              rv[0], rv[1], rv[2])
                else:
                    # 无可移动目标
                    pass

            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)
            self._update_object_status()

        except BlockingIOError:
            pass
        except Exception as e:
            self.status_label.setText(f"错误: {e}")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        viewport = self._get_viewport()
        cam_label = self._camera_label(viewport) if viewport else "???"

        mode = "🎥相机" if self.driver_enabled else "🧰物体"
        obj_info = ""
        if not self.driver_enabled:
            target = self._get_movable_target()
            if target is not None:
                obj_info = f" | {target.name()}"
                # 如果是 OBJ 节点, 显示世界位置
                if hasattr(target, 'worldTransform') and \
                        callable(getattr(target, 'worldTransform', None)):
                    try:
                        om = mat4_to_numpy(target.worldTransform())
                        op = om[3, :3]
                        obj_info += f" pos({op[0]:.3f},{op[1]:.3f},{op[2]:.3f})"
                    except Exception:
                        pass
            else:
                obj_info = " | 无可移动物体"

        self.status_label.setText(
            f"{mode} | {cam_label}{obj_info}\n"
            f"in  T:({tx:+4d},{ty:+4d},{tz:+4d})  R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
            f"out T:({tv[0]:+.5f},{tv[1]:+.5f},{tv[2]:+.5f})  "
            f"R:({rv[0]:+.4f}°,{rv[1]:+.4f}°,{rv[2]:+.4f}°)")

    # ── 物体操控 ──────────────────────────────────────────

    def _get_movable_target(self):
        """获取当前视口中可移动的目标节点, 无则返回 None

        已确认兼容:
          - OBJ 节点 (cam, geo, null, bone 等)
          - Transform SOP / Edit SOP (有 tx/ty/tz)
          - Rig Pose (有 group 参数 + transform 子参数)
          - Bone / Capture 相关
        """
        try:
            sel = hou.selectedNodes()
            if not sel:
                return None
            node = sel[0]

            # OBJ 层级 — 通用
            if (hasattr(node, 'worldTransform') and
                    callable(getattr(node, 'worldTransform', None))):
                return node

            # SOP 层级 — 任意被选中的 SOP 节点都可能被 Enter 激活 handle
            cat = node.type().category()
            if cat in (hou.sopNodeTypeCategory(),):
                return node  # 接受所有 SOP, _move_sop_node 内部按参数适配

        except Exception:
            pass
        return None

    def _get_node_parent_obj(self, node):
        """找到节点的父 OBJ 容器 (有 worldTransform 的祖先)"""
        try:
            if (hasattr(node, 'worldTransform') and
                    callable(getattr(node, 'worldTransform', None))):
                return node
            parent = node.parent()
            if parent is not None:
                return self._get_node_parent_obj(parent)
        except Exception:
            pass
        return None

    def move_selected_object(self, node, tx, ty, tz, rx, ry, rz):
        """通用物体操控 — 全部基于物体本地坐标系

        OBJ 层级: 沿 OBJ 自身轴平移, 绕自身轴旋转
        SOP 层级: 沿父 OBJ 局部轴平移, 绕 SOP 参数轴旋转
        """
        try:
            is_obj = (hasattr(node, 'worldTransform') and
                      callable(getattr(node, 'worldTransform', None)))

            if is_obj:
                self._move_obj_node(node, tx, ty, tz, rx, ry, rz)
            else:
                self._move_sop_node(node, tx, ty, tz, rx, ry, rz)

        except Exception as e:
            self.status_label.setText(f"物体移动错误: {e}")

    def _move_obj_node(self, node, tx, ty, tz, rx, ry, rz):
        """OBJ 层级: 沿自身局部轴移动 + 绕自身轴旋转"""
        obj_mat = mat4_to_numpy(node.worldTransform())
        obj_pos = obj_mat[3, :3].copy()
        obj_rot = obj_mat[:3, :3].copy()

        # 平移 — OBJ 局部轴
        # Row 0=right, Row 1=up, Row 2=fwd (Houdini row-major)
        obj_right = obj_rot[0, :]
        obj_up    = obj_rot[1, :]
        obj_fwd   = obj_rot[2, :]
        obj_pos += tx * obj_right + ty * obj_fwd + tz * obj_up

        # 旋转 — OBJ 局部轴 (Pitch绕X, Roll绕Z, Yaw绕Y)
        if abs(rz) > 1e-10:            # Yaw: 绕 OBJ 的 up(Y)
            R = rodrigues(obj_rot[1, :], np.radians(rz))
            obj_rot = obj_rot @ R.T
        if abs(rx) > 1e-10:            # Pitch: 绕 OBJ 的右(X)
            R = rodrigues(obj_rot[0, :], np.radians(rx))
            obj_rot = obj_rot @ R.T
        if abs(ry) > 1e-10:            # Roll: 绕 OBJ 的前(Z)
            R = rodrigues(obj_rot[2, :], np.radians(ry))
            obj_rot = obj_rot @ R.T

        obj_rot = orthonormalize(obj_rot)
        node.setWorldTransform(numpy_to_mat4(obj_pos, obj_rot))

    def _move_sop_node(self, node, tx, ty, tz, rx, ry, rz):
        """SOP 层级: 多模式参数适配

        尝试顺序:
          1. tx/ty/tz (Transform SOP, Edit SOP, Null)
          2. px/py/pz (某些 bone/rig 节点)
          3. t[x/y/z] 单参 (通用)
          4. 直接读 parmTuple('t') (Transform SOP 标准)
        """
        moved = False

        # ── 平移 ──
        # 尝试 parmTuple('t') — 最通用的方式
        t_tuple = node.parmTuple('t')
        if t_tuple is not None and len(t_tuple) >= 3:
            vals = t_tuple.eval()
            node.parmTuple('t').set((vals[0] + tx, vals[1] + ty, vals[2] + tz))
            moved = True
        elif node.parm('tx') is not None:
            node.parm('tx').set(node.parm('tx').eval() + tx)
            node.parm('ty').set(node.parm('ty').eval() + ty)
            node.parm('tz').set(node.parm('tz').eval() + tz)
            moved = True
        elif node.parm('px') is not None:
            node.parm('px').set(node.parm('px').eval() + tx)
            node.parm('py').set(node.parm('py').eval() + ty)
            node.parm('pz').set(node.parm('pz').eval() + tz)
            moved = True

        # ── 旋转 ──
        r_tuple = node.parmTuple('r')
        if r_tuple is not None and len(r_tuple) >= 3:
            r_vals = r_tuple.eval()
            nrx, nry, nrz = r_vals[0], r_vals[1], r_vals[2]
            if abs(rx) > 1e-10: nrx += rx
            if abs(ry) > 1e-10: nry += ry
            if abs(rz) > 1e-10: nrz += rz
            node.parmTuple('r').set((nrx, nry, nrz))
            moved = True
        elif node.parm('rx') is not None:
            if abs(rx) > 1e-10: node.parm('rx').set(node.parm('rx').eval() + rx)
            if abs(ry) > 1e-10: node.parm('ry').set(node.parm('ry').eval() + ry)
            if abs(rz) > 1e-10: node.parm('rz').set(node.parm('rz').eval() + rz)
            moved = True

        if not moved:
            self.status_label.setText(
                f"SOP 节点 {node.name()} 无可写变换参数")

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
