# spacemouse_receiver.pypanel
"""
SpaceMouse Receiver -- viewport object manipulator
==================================================
Device: 3Dconnexion SpaceExplorer (VID:046D PID:C627)
Axis mapping (SpaceExplorer):
  Tx: left(-)/right(+)      Rx: Pitch  forward(-)/backward(+)
  Ty: fwd(-)/back(+)        Ry: Roll   cw(-)/ccw(+)
  Tz: down(+)/up(-)         Rz: Yaw    cw(-)/ccw(+) (top view)

Houdini matrix (row-major): v_world = v_local * M
  Translate at Row 3 | Row 0=right Row 1=up Row 2=fwd
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

DRIVER_CFG = os.path.join(
    os.environ.get('APPDATA', ''),
    r'3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml')


# ======================================================================
# Matrix utilities
# ======================================================================

def mat4_to_numpy(m):
    """hou.Matrix4 -> numpy 4x4 (float64)"""
    return np.array([
        [m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3)],
        [m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3)],
        [m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3)],
        [m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3)],
    ], dtype=np.float64)


def numpy_to_mat4(pos, rot):
    """pos(3,) + rot(3,3) -> hou.Matrix4"""
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
    """numpy 3x3 -> hou.Matrix3"""
    m = hou.Matrix3()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
    return m


def rodrigues(axis, angle_rad):
    """Rotation 3x3 about arbitrary axis (column-vector convention)"""
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
    """Light orthogonalization -- preserves forward (row2)"""
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


# ======================================================================
# Main panel
# ======================================================================

class SpaceMouseReceiver(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.setblocking(False)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_spacemouse)

        self.active = False
        self.init_ui()

    # -- UI -----------------------------------------------------------

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        self.status_label = QtWidgets.QLabel("Waiting for SpaceMouse data...")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(110)
        self.status_label.setStyleSheet(
            "QLabel { font-family: Consolas, monospace; font-size: 12px;"
            " background: #1e1e1e; color: #d4d4d4; padding: 8px;"
            " border: 1px solid #444; border-radius: 4px; }")
        layout.addWidget(self.status_label)

        btn_row = QtWidgets.QHBoxLayout()

        self.toggle_btn = QtWidgets.QPushButton("Start")
        self.toggle_btn.clicked.connect(self.toggle)
        self.toggle_btn.setMinimumHeight(30)
        btn_row.addWidget(self.toggle_btn)

        self.detect_btn = QtWidgets.QPushButton("Detect Parms")
        self.detect_btn.clicked.connect(self.print_scoped_parms)
        self.detect_btn.setMinimumHeight(30)
        btn_row.addWidget(self.detect_btn)

        layout.addLayout(btn_row)

        # Driver toggle
        drv_row = QtWidgets.QHBoxLayout()
        drv_row.addWidget(QtWidgets.QLabel("3DxWare:"))

        self.driver_btn = QtWidgets.QPushButton("Driver: ON")
        self.driver_btn.clicked.connect(self.toggle_driver)
        self.driver_btn.setMinimumHeight(30)
        self.driver_btn.setToolTip(
            "Toggle 3DxWare axis enable/disable for Houdini\n"
            "ON: driver controls viewport camera\n"
            "OFF: we control selected objects")
        drv_row.addWidget(self.driver_btn)

        self.driver_enabled = True
        self._write_driver_enabled(True)
        self._update_driver_btn()

        layout.addLayout(drv_row)

        # Sensitivity
        sens = QtWidgets.QGroupBox("Sensitivity (raw / 350 * gain)")
        sens_row = QtWidgets.QHBoxLayout()

        sens_row.addWidget(QtWidgets.QLabel("T:"))
        self.t_spin = QtWidgets.QDoubleSpinBox()
        self.t_spin.setRange(0.0001, 10.0)
        self.t_spin.setValue(0.05)
        self.t_spin.setSingleStep(0.005)
        self.t_spin.setDecimals(5)
        sens_row.addWidget(self.t_spin)

        sens_row.addSpacing(12)

        sens_row.addWidget(QtWidgets.QLabel("R deg:"))
        self.r_spin = QtWidgets.QDoubleSpinBox()
        self.r_spin.setRange(0.0001, 10.0)
        self.r_spin.setValue(1.0)
        self.r_spin.setSingleStep(0.1)
        self.r_spin.setDecimals(5)
        sens_row.addWidget(self.r_spin)

        sens_row.addStretch()
        sens.setLayout(sens_row)
        layout.addWidget(sens)

        # Per-axis gain
        gain = QtWidgets.QGroupBox("Per-axis gain (+/-1.0, negative=invert)")
        gain_lay = QtWidgets.QGridLayout()

        t_labels = ["Tx (L/R)", "Ty (F/B)", "Tz (D/U)"]
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

    # -- Start/Stop ---------------------------------------------------

    def toggle(self):
        if self.active:
            self.timer.stop()
            self.toggle_btn.setText("Start")
            self.active = False
            self.status_label.setText("Stopped")
        else:
            self._drain_udp()  # discard stale packets
            self.timer.start(4)
            self.toggle_btn.setText("Stop")
            self.active = True
            self.status_label.setText("Running, waiting for data...")

    def _drain_udp(self):
        """Discard all buffered UDP packets before starting"""
        try:
            while True:
                self.sock.recvfrom(1024)
        except BlockingIOError:
            pass

    # -- Driver toggle ------------------------------------------------

    def toggle_driver(self):
        new_state = not self.driver_enabled
        if self._write_driver_enabled(new_state):
            self.driver_enabled = new_state
            self._update_driver_btn()

    def _update_driver_btn(self):
        if self.driver_enabled:
            self.driver_btn.setText("Driver: ON (Camera)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #2a622a; color: #fff; }")
        else:
            self.driver_btn.setText("Driver: OFF (Object)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #6a4a1a; color: #fff; }")

    def _write_driver_enabled(self, enable):
        """Edit SideFX_HoudiniFX.xml <Enabled> values, driver reloads live"""
        try:
            if not os.path.exists(DRIVER_CFG):
                self.status_label.setText(f"Config not found:\n{DRIVER_CFG}")
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
                    f"Driver: {'enabled' if enable else 'disabled'} "
                    f"{changed} axes -> written\n"
                    f"3DxWare watches XML, takes effect ~1 sec")
            else:
                self.status_label.setText(
                    f"Driver: axes already {'enabled' if enable else 'disabled'}")

            return True

        except Exception as e:
            self.status_label.setText(f"Driver config write failed: {e}")
            return False

    # -- Detect parms -------------------------------------------------

    def print_scoped_parms(self):
        """Print current scoped / active transform parameters"""
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
                info += "state attrs:\n"
                for attr in dir(state):
                    if not attr.startswith('_'):
                        try:
                            val = getattr(state, attr)
                            if callable(val):
                                continue
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
        except Exception:
            import traceback
            info += f"\nERROR:\n{traceback.format_exc()}"

        print(info)
        self.status_label.setText(info)

    # -- Viewport helpers ---------------------------------------------

    def _get_viewport(self):
        try:
            desktop = hou.ui.curDesktop()
            viewer = desktop.paneTabOfType(hou.paneTabType.SceneViewer)
            if viewer is None:
                return None
            return viewer.curViewport()
        except Exception:
            return None

    def _camera_label(self, viewport):
        cam_node = viewport.camera()
        if cam_node is not None:
            return f"[Cam] {cam_node.path()}"
        return "[No Cam]"

    # -- UDP receive + move -------------------------------------------

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

            # Axis mapping: SpaceMouse -> Houdini (Y-up, Z-fwd)
            #   Tx(L-/R+) -> -X    (inverted)
            #   Ty(F-/B+) -> -Z
            #   Tz(D+/U-) -> -Y
            hx, hy, hz = -tv[0], -tv[2], -tv[1]

            # Rotation: Pitch->-X  Roll->-Z  Yaw->Y
            hrx, hry, hrz = -rv[0], rv[2], -rv[1]

            if not self.driver_enabled:
                target = self._get_movable_target()
                if target is not None:
                    self._move_target(target, hx, hy, hz, hrx, hry, hrz)
            # else: driver on -> official driver handles camera

            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)

        except BlockingIOError:
            pass
        except Exception:
            import traceback
            traceback.print_exc()
            self.status_label.setText("Error: see console")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        viewport = self._get_viewport()
        cam_label = self._camera_label(viewport) if viewport else "???"
        mode = "[Object]" if not self.driver_enabled else "[Camera]"

        hx, hy, hz = -tv[0], -tv[2], -tv[1]
        info = (
            f"{mode} | {cam_label}\n"
            f"raw T:({tx:+4d},{ty:+4d},{tz:+4d})  R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
            f"Hou  X:{hx:+.5f} Y:{hy:+.5f} Z:{hz:+.5f}  "
            f"R:({rv[0]:+.4f},{rv[1]:+.4f},{rv[2]:+.4f})")

        try:
            sel = hou.selectedNodes()
            if sel:
                scoped = [p for p in sel[0].parms() if p.isScoped()]
                if scoped:
                    info += f"\nCH: {len(scoped)} scoped parms"
        except Exception:
            pass

        self.status_label.setText(info)

    # -- Object manipulation ------------------------------------------

    def _get_apex_state(self):
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
        # 1. Selected node + isScoped (no hou.playbar.channelList!)
        try:
            sel = hou.selectedNodes()
            if sel:
                node = sel[0]

                # OBJ level
                if hasattr(node, 'worldTransform') and \
                        callable(getattr(node, 'worldTransform', None)):
                    self._log_target('obj', node.name())
                    return ('obj', node)

                # SOP: scoped transform parms
                scoped = [p for p in node.parms() if p.isScoped()]
                if scoped:
                    self._log_target('ch', f'{len(scoped)} scoped on {node.name()}')
                    return ('ch', scoped)

                # LOP / any: direct t/r parm tuples
                t_tuple = node.parmTuple('t')
                if t_tuple is not None and len(t_tuple) >= 3:
                    self._log_target('parm', f't/r on {node.name()}')
                    return ('parm', node)
        except Exception:
            pass

        # 2. APEX state -- use control_selection (not control_paths!)
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
            print(f"[SpaceMouse] target: [{kind}] {detail}")
            self._last_target_kind = kind

    def _move_target(self, target, tx, ty, tz, rx, ry, rz):
        kind = target[0]
        if kind == 'ch':
            self._move_ch_parms(target[1], tx, ty, tz, rx, ry, rz)
        elif kind == 'parm':
            self._move_parm_node(target[1], tx, ty, tz, rx, ry, rz)
        elif kind == 'apex':
            self._move_apex_controls(target[1], tx, ty, tz, rx, ry, rz)
        elif kind == 'obj':
            self._move_obj_node(target[1], tx, ty, tz, rx, ry, rz)

    # Regex patterns for t{n}x / r{n}x (Rig Pose + Transform SOP)
    _RE_TX = re.compile(r'(?:^|[_:])t\d*x$')
    _RE_TY = re.compile(r'(?:^|[_:])t\d*y$')
    _RE_TZ = re.compile(r'(?:^|[_:])t\d*z$')
    _RE_RX = re.compile(r'(?:^|[_:])r\d*x$')
    _RE_RY = re.compile(r'(?:^|[_:])r\d*y$')
    _RE_RZ = re.compile(r'(?:^|[_:])r\d*z$')

    _ch_printed = False

    def _move_ch_parms(self, parms, tx, ty, tz, rx, ry, rz):
        """Channel List scoped parms -- regex match t{n}x/r{n}x format"""
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
            print(f"[SpaceMouse] T:{tx_name} {ty_name} {tz_name} | R:{rx_name} {ry_name} {rz_name}")
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
        """APEX animate state -- via graph_parms"""
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

    def _move_parm_node(self, node, tx, ty, tz, rx, ry, rz):
        """LOP/SOP with t/r parm tuples -- direct write"""
        t_tuple = node.parmTuple('t')
        if t_tuple and len(t_tuple) >= 3:
            vals = t_tuple.eval()
            t_tuple.set((vals[0] + tx, vals[1] + ty, vals[2] + tz))
        r_tuple = node.parmTuple('r')
        if r_tuple and len(r_tuple) >= 3:
            vals = r_tuple.eval()
            nrx, nry, nrz = vals[0], vals[1], vals[2]
            if abs(rx) > 1e-10: nrx += rx
            if abs(ry) > 1e-10: nry += ry
            if abs(rz) > 1e-10: nrz += rz
            r_tuple.set((nrx, nry, nrz))

    def _move_obj_node(self, node, tx, ty, tz, rx, ry, rz):
        """OBJ node -- setWorldTransform"""
        obj_mat = mat4_to_numpy(node.worldTransform())
        obj_pos = obj_mat[3, :3].copy()
        obj_rot = obj_mat[:3, :3].copy()

        # Houdini axes: X=right(row0), Y=up(row1), Z=fwd(row2)
        obj_pos += tx * obj_rot[0, :] + ty * obj_rot[1, :] + tz * obj_rot[2, :]

        # Rotation: rx=X(pitch), ry=Y(yaw), rz=Z(roll)
        if abs(ry) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0., 1., 0.]), np.radians(ry)).T
        if abs(rx) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([1., 0., 0.]), np.radians(rx)).T
        if abs(rz) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0., 0., 1.]), np.radians(rz)).T

        obj_rot = orthonormalize(obj_rot)
        node.setWorldTransform(numpy_to_mat4(obj_pos, obj_rot))


def createInterface():
    return SpaceMouseReceiver()
