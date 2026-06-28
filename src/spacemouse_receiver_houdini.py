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
import re
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

        self.detect_btn = QtWidgets.QPushButton("🔍 检测参数")
        self.detect_btn.clicked.connect(self.print_scoped_parms)
        self.detect_btn.setMinimumHeight(30)
        btn_row.addWidget(self.detect_btn)

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

        # 初始状态: 驱动开 (相机模式) — 强制写入以确保上次退出时残留的关闭状态被清除
        self.driver_enabled = True
        self._write_driver_enabled(True)
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

    # ── 检测参数 ─────────────────────────────────────────

    def print_scoped_parms(self):
        """诊断: 只测 APEX state 路径"""
        info = ""
        try:
            import apex
            sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
            info += f"SceneViewer: {'OK' if sv else 'None'}\n"

            kwargs = {}
            sv.runStateCommand('getState', kwargs)
            state = kwargs.get('state')
            info += f"state: {'OK' if state else 'None'}\n"

            if state:
                # dump state 的全部属性
                info += "state attrs:\n"
                for attr in dir(state):
                    if not attr.startswith('_'):
                        try:
                            val = getattr(state, attr)
                            if callable(val): continue
                            s = str(val)[:80]
                            info += f"  .{attr} = {s}\n"
                        except Exception:
                            pass

                ctrls = getattr(state, 'control_paths', None)
                info += f"\ncontrol_paths: {ctrls}\n"

                if ctrls:
                    from apex.control_2 import controlRigPath
                    scene = state.scene
                    for cp in ctrls[:3]:
                        info += f"\n--- {cp} ---\n"
                        rp = controlRigPath(cp)
                        info += f"rig_path: {rp}\n"
                        rig = scene.getData(rp)
                        info += f"rig: {'OK' if rig else 'None'}\n"
                        cm = scene.getData(f"{rp}/control_manager")
                        info += f"ctrl_mgr: {'OK' if cm else 'None'}\n"
                        if cm and rig:
                            cpm = cm.getControlMapping(cp)
                            info += f"  .t = {cpm.t}\n"
                            info += f"  .r = {cpm.r}\n"
                            if cpm.t:
                                info += f"  graph_parms[{cpm.t}] = {rig.graph_parms.get(cpm.t)}\n"
        except Exception as ex:
            import traceback
            info += f"\nERROR:\n{traceback.format_exc()}"

        print(info)
        self.status_label.setText(info)

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

            if not self.driver_enabled:
                # 驱动关 → 物体模式: Channel List > OBJ > SOP
                target = self._get_movable_target()
                if target is not None:
                    self._move_target(target, tv[0], tv[1], tv[2],
                                      rv[0], rv[1], rv[2])
                # 无可移动目标 → 不动
            else:
                # 驱动开 → 官方驱动管相机, 我们只显示
                pass

            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)

        except BlockingIOError:
            pass
        except Exception:
            import traceback
            traceback.print_exc()
            self.status_label.setText("错误: 见控制台")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        viewport = self._get_viewport()
        cam_label = self._camera_label(viewport) if viewport else "???"
        mode = "🧰物体" if not self.driver_enabled else "🎥相机"

        info = (
            f"{mode} | {cam_label}\n"
            f"in  T:({tx:+4d},{ty:+4d},{tz:+4d})  R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
            f"out T:({tv[0]:+.5f},{tv[1]:+.5f},{tv[2]:+.5f})  "
            f"R:({rv[0]:+.4f}°,{rv[1]:+.4f}°,{rv[2]:+.4f}°)")

        # 显示 scoped 参数数
        try:
            sel = hou.selectedNodes()
            if sel:
                scoped = [p for p in sel[0].parms() if p.isScoped()]
                if scoped:
                    info += f"\nCH: {len(scoped)} scoped parms"
        except Exception:
            pass

        self.status_label.setText(info)

    # ── 物体操控 ──────────────────────────────────────────

    def _get_apex_state(self):
        """获取 APEX animate state (失败返回 None)"""
        try:
            import apex
            sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
            if sv is None:
                return None
            kwargs = {}
            sv.runStateCommand('getState', kwargs)
            return kwargs.get('state')
        except Exception:
            return None

    _last_target_kind = None

    def _get_movable_target(self):
        """获取可移动目标 — 优先从选中节点 scoped parms, 其次 APEX state, 最后 OBJ"""
        # 1. 选中节点 + isScoped (不依赖 hou.playbar.channelList!)
        try:
            sel = hou.selectedNodes()
            if sel:
                node = sel[0]

                # OBJ 层级优先
                if hasattr(node, 'worldTransform') and \
                        callable(getattr(node, 'worldTransform', None)):
                    self._log_target('obj', node.name())
                    return ('obj', node)

                # SOP 层级: 检查是否有 scoped 的变换参数
                scoped = [p for p in node.parms() if p.isScoped()]
                if scoped:
                    self._log_target('ch', f'{len(scoped)} scoped on {node.name()}')
                    return ('ch', scoped)
        except Exception:
            pass

        # 2. APEX state — 用 control_selection (非 control_paths!)
        try:
            state = self._get_apex_state()
            if state is not None:
                ctrls = getattr(state, 'control_selection', None)
                if ctrls:
                    self._log_target('apex', f'{len(ctrls)} controls')
                    return ('apex', (state, list(ctrls)))
        except Exception:
            pass

        self._last_target_kind = None
        return None

    def _log_target(self, kind, detail):
        if self._last_target_kind != kind:
            print(f"[SpaceMouse] 目标: [{kind}] {detail}")
            self._last_target_kind = kind

    def _move_target(self, target, tx, ty, tz, rx, ry, rz):
        """分发到对应处理器"""
        kind = target[0]
        if kind == 'ch':
            self._move_ch_parms(target[1], tx, ty, tz, rx, ry, rz)
        elif kind == 'apex':
            self._move_apex_controls(target[1], tx, ty, tz, rx, ry, rz)
        elif kind == 'obj':
            self._move_obj_node(target[1], tx, ty, tz, rx, ry, rz)

    # 匹配 Rig Pose 't0x' / Transform SOP 'tx' 等格式
    _RE_TX = re.compile(r'(?:^|[_:])t\d*x$')
    _RE_TY = re.compile(r'(?:^|[_:])t\d*y$')
    _RE_TZ = re.compile(r'(?:^|[_:])t\d*z$')
    _RE_RX = re.compile(r'(?:^|[_:])r\d*x$')
    _RE_RY = re.compile(r'(?:^|[_:])r\d*y$')
    _RE_RZ = re.compile(r'(?:^|[_:])r\d*z$')

    _ch_printed = False  # 只打印一次匹配结果

    def _move_ch_parms(self, parms, tx, ty, tz, rx, ry, rz):
        """Channel List scoped 参数 — 正则匹配 t{n}x / r{n}x 等格式"""
        t_map = [None, None, None]
        r_map = [None, None, None]

        for p in parms:
            name = p.name().lower()
            if SpaceMouseReceiver._RE_TX.search(name): t_map[0] = p
            elif SpaceMouseReceiver._RE_TY.search(name): t_map[1] = p
            elif SpaceMouseReceiver._RE_TZ.search(name): t_map[2] = p
            if SpaceMouseReceiver._RE_RX.search(name): r_map[0] = p
            elif SpaceMouseReceiver._RE_RY.search(name): r_map[1] = p
            elif SpaceMouseReceiver._RE_RZ.search(name): r_map[2] = p

        if not SpaceMouseReceiver._ch_printed:
            tx_name = t_map[0].name() if t_map[0] else '-'
            ty_name = t_map[1].name() if t_map[1] else '-'
            tz_name = t_map[2].name() if t_map[2] else '-'
            rx_name = r_map[0].name() if r_map[0] else '-'
            ry_name = r_map[1].name() if r_map[1] else '-'
            rz_name = r_map[2].name() if r_map[2] else '-'
            print(f"[SpaceMouse] T matched: {tx_name} {ty_name} {tz_name} | R matched: {rx_name} {ry_name} {rz_name}")
            SpaceMouseReceiver._ch_printed = True

        if all(t_map):
            t_map[0].set(t_map[0].eval() + tx)
            t_map[1].set(t_map[1].eval() + ty)
            t_map[2].set(t_map[2].eval() + tz)
        if all(r_map):
            if abs(rx) > 1e-10: r_map[0].set(r_map[0].eval() + rx)
            if abs(ry) > 1e-10: r_map[1].set(r_map[1].eval() + ry)
            if abs(rz) > 1e-10: r_map[2].set(r_map[2].eval() + rz)

    _apex_printed = False

    def _move_apex_controls(self, target, tx, ty, tz, rx, ry, rz):
        """APEX animate state — 用 state.control_manager + graph_parms"""
        state, ctrl_paths = target
        if not ctrl_paths:
            return

        try:
            from apex.control_2 import controlRigPath
            ctrl_mgr = state.control_manager
            scene = state.scene

            for ctrl_path in ctrl_paths:
                rig_path = controlRigPath(ctrl_path)
                rig = scene.getData(rig_path)
                if rig is None:
                    continue

                ctrl_map = ctrl_mgr.getControlMapping(ctrl_path)

                if not SpaceMouseReceiver._apex_printed:
                    print(f"[SpaceMouse] APEX ctrl={ctrl_path} "
                          f"t={ctrl_map.t} r={ctrl_map.r}")
                    SpaceMouseReceiver._apex_printed = True

                if ctrl_map.t:
                    cur = rig.graph_parms.get(ctrl_map.t)
                    if cur is None:
                        cur = hou.Vector3(0, 0, 0)
                    cur = hou.Vector3(cur) if not isinstance(cur, hou.Vector3) else cur
                    rig.graph_parms[ctrl_map.t] = hou.Vector3(
                        cur[0] + tx, cur[1] + ty, cur[2] + tz)

                if ctrl_map.r and (abs(rx) > 1e-10 or abs(ry) > 1e-10 or abs(rz) > 1e-10):
                    cur = rig.graph_parms.get(ctrl_map.r)
                    if cur is None:
                        cur = hou.Vector3(0, 0, 0)
                    cur = hou.Vector3(cur) if not isinstance(cur, hou.Vector3) else cur
                    rig.graph_parms[ctrl_map.r] = hou.Vector3(
                        cur[0] + rx, cur[1] + ry, cur[2] + rz)

            state.runSceneCallbacks()

        except Exception:
            import traceback
            traceback.print_exc()

    def _move_obj_node(self, node, tx, ty, tz, rx, ry, rz):
        """OBJ 节点 — setWorldTransform"""
        obj_mat = mat4_to_numpy(node.worldTransform())
        obj_pos = obj_mat[3, :3].copy()
        obj_rot = obj_mat[:3, :3].copy()

        obj_pos += tx * obj_rot[0, :] + ty * obj_rot[2, :] + tz * obj_rot[1, :]

        if abs(rz) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0., 1., 0.]), np.radians(rz)).T
        if abs(rx) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([1., 0., 0.]), np.radians(rx)).T
        if abs(ry) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0., 0., 1.]), np.radians(ry)).T

        obj_rot = orthonormalize(obj_rot)
        node.setWorldTransform(numpy_to_mat4(obj_pos, obj_rot))



def createInterface():
    return SpaceMouseReceiver()
