# spacemouse_houdini.py
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
import os
import xml.etree.ElementTree as ET
import numpy as np
import hid
import struct
import threading
from PySide6 import QtCore, QtWidgets

VENDOR_3DCONNEXION = 0x046D
PRODUCT_SPACEEXPLORER = 0xC627

DRIVER_CFG = os.path.join(
    os.environ.get('APPDATA', ''),
    r'3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml')

# Debug log
_DEBUG_DIR = r'd:\code\dev\Houdini\spaceMouse1\debug'
_DEBUG_LOG = os.path.join(_DEBUG_DIR, 'spacemouse_debug.log')
_ORIG_PRINT = print


def _tee_print(*args, **kwargs):
    _ORIG_PRINT(*args, **kwargs)
    try:
        if not os.path.exists(_DEBUG_DIR):
            os.makedirs(_DEBUG_DIR)
        import io
        buf = io.StringIO()
        _ORIG_PRINT(*args, file=buf, **kwargs)
        with open(_DEBUG_LOG, 'a', encoding='utf-8') as f:
            f.write(buf.getvalue())
    except Exception:
        pass


print = _tee_print  # noqa: A001

try:
    if not os.path.exists(_DEBUG_DIR):
        os.makedirs(_DEBUG_DIR)
    with open(_DEBUG_LOG, 'w', encoding='utf-8') as f:
        f.write('=== SpaceMouse Debug Log ===\n')
except Exception:
    pass


# ======================================================================
# Matrix utilities
# ======================================================================

def mat4_to_numpy(m):
    return np.array([
        [m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3)],
        [m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3)],
        [m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3)],
        [m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3)],
    ], dtype=np.float64)


def numpy_to_mat4(pos, rot):
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
    m = hou.Matrix3()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
    return m


def rodrigues(axis, angle_rad):
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


def tuple_to_mat4(t, col_major=False):
    """Convert 16-tuple to 4x4 numpy matrix. Handles both row and column major."""
    m = np.array(t, dtype=np.float64).reshape(4, 4)
    if col_major:
        m = m.T
    return m


# ======================================================================
# SpaceMouse HID Reader
# ======================================================================

class SpaceMouseReader:
    """Direct HID reader for SpaceMouse device"""
    def __init__(self):
        self.dev = None
        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0]
        self.button_state = 0
        self.running = False
        self.frame_count = 0
        self.lock = threading.Lock()
        self.read_thread = None
        self.data_callback = None

    def connect(self):
        devices = [d for d in hid.enumerate()
                   if d['vendor_id'] == VENDOR_3DCONNEXION
                   and d['product_id'] == PRODUCT_SPACEEXPLORER
                   and d.get('usage_page') == 1
                   and d.get('usage') == 8]

        if not devices:
            raise RuntimeError("SpaceExplorer not found")

        self.dev = hid.device()
        self.dev.open_path(devices[0]['path'])
        self.dev.set_nonblocking(0)
        print(f"Connected: {self.dev.get_product_string()}")

    def start_reading(self, callback):
        """Start reading thread with callback for new data"""
        self.data_callback = callback
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        while self.running:
            try:
                data = self.dev.read(64, timeout_ms=1)
                if not data:
                    continue

                report_id = data[0]
                should_notify = False

                with self.lock:
                    # Report 1: Translation (Tx, Ty, Tz)
                    if report_id == 1 and len(data) >= 7:
                        self.translation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.translation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.translation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_notify = True

                    # Report 2: Rotation (Rx, Ry, Rz)
                    elif report_id == 2 and len(data) >= 7:
                        self.rotation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.rotation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.rotation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_notify = True

                    # Report 3: Button state
                    elif report_id == 3 and len(data) >= 4:
                        self.button_state = (data[1] |
                                            (data[2] << 8) |
                                            (data[3] << 16))
                        should_notify = True

                    if should_notify and self.data_callback:
                        self.data_callback(
                            self.translation[:],
                            self.rotation[:],
                            self.button_state
                        )
                        self.frame_count += 1

            except Exception as e:
                if self.running:
                    print(f"Read error: {e}")

    def get_current_data(self):
        """Thread-safe read of current state"""
        with self.lock:
            return (self.translation[:], self.rotation[:], self.button_state)

    def close(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.dev:
            self.dev.close()


# ======================================================================
# Main panel
# ======================================================================

class SpaceMouseReceiver(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.reader = SpaceMouseReader()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.process_spacemouse_data)

        self.active = False
        self.destroyed.connect(self._cleanup)
        
        # Current data storage (updated by HID callback)
        self.current_translation = [0, 0, 0]
        self.current_rotation = [0, 0, 0]
        self.current_buttons = 0
        self.data_lock = threading.Lock()
        
        self.init_ui()

    def _cleanup(self):
        """Release device on panel close"""
        try:
            if hasattr(self, 'reader') and self.reader:
                self.reader.close()
        except Exception:
            pass

    def _on_spacemouse_data(self, translation, rotation, buttons):
        """Callback from HID reader thread"""
        with self.data_lock:
            self.current_translation = translation
            self.current_rotation = rotation
            self.current_buttons = buttons

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

        drv_row = QtWidgets.QHBoxLayout()
        drv_row.addWidget(QtWidgets.QLabel("3DxWare:"))
        self.driver_btn = QtWidgets.QPushButton("Driver: ON (Camera)")
        self.driver_btn.clicked.connect(self.toggle_driver)
        self.driver_btn.setMinimumHeight(30)
        drv_row.addWidget(self.driver_btn)

        # Camera-space toggle
        self.space_btn = QtWidgets.QPushButton("Cam Space")
        self.space_btn.clicked.connect(self.toggle_space_mode)
        self.space_btn.setMinimumHeight(30)
        self.space_btn.setMinimumWidth(72)
        self.space_btn.setStyleSheet("QPushButton { background: #2a4a6a; color: #fff; }")
        self.space_btn.setToolTip("Toggle movement space\nCam = camera-relative\nAbs = world absolute")
        drv_row.addWidget(self.space_btn)

        self.camera_space = True  # default: camera-relative
        self.driver_enabled = True
        self._write_driver_enabled(True)
        self._update_driver_btn()
        layout.addLayout(drv_row)

        # Channel toggle buttons + T/R master + All/None, right-aligned
        ch_row = QtWidgets.QHBoxLayout()
        ch_row.addStretch()
        ch_labels = ['Tx', 'Ty', 'Tz', 'Rx', 'Ry', 'Rz']
        ch_colors = ['#c44', '#4c4', '#44c', '#c44', '#4c4', '#44c']
        self.ch_btns = []
        self.ch_active = [True] * 6
        self.ch_solo = None
        self.ch_prev = [True] * 6
        for i in range(6):
            btn = QtWidgets.QPushButton(ch_labels[i])
            btn.setMinimumHeight(22)
            btn.setMinimumWidth(40)
            btn.setMaximumWidth(46)
            btn.setStyleSheet(f"QPushButton {{ background: {ch_colors[i]}; color: #fff; font-size: 9pt; padding: 1px 4px; }}")
            btn.setToolTip(f"{ch_labels[i]}\nClick: toggle on/off\nAlt+Click: solo/unsolo")
            btn.clicked.connect(lambda _=None, idx=i: self._on_ch_toggle(idx))
            self.ch_btns.append(btn)
            ch_row.addWidget(btn)
        ch_row.addSpacing(4)
        # T / R master toggles
        self.t_master_btn = QtWidgets.QPushButton("T")
        self.t_master_btn.setMinimumHeight(22); self.t_master_btn.setMaximumWidth(28)
        self.t_master_btn.clicked.connect(lambda: self._ch_toggle_group(0))
        ch_row.addWidget(self.t_master_btn)
        self.r_master_btn = QtWidgets.QPushButton("R")
        self.r_master_btn.setMinimumHeight(22); self.r_master_btn.setMaximumWidth(28)
        self.r_master_btn.clicked.connect(lambda: self._ch_toggle_group(3))
        ch_row.addWidget(self.r_master_btn)
        # All / None
        for label, cb in [('All', lambda: self._ch_set_all(True)),
                           ('None', lambda: self._ch_set_all(False))]:
            btn = QtWidgets.QPushButton(label)
            btn.setMinimumHeight(22); btn.setMaximumWidth(40)
            btn.setStyleSheet("QPushButton { background: #555; color: #ccc; font-size: 9pt; padding: 1px 4px; }")
            btn.clicked.connect(lambda _=None, f=cb: f())
            ch_row.addWidget(btn)
        self._update_ch_buttons()
        layout.addLayout(ch_row)

        # Sensitivity + Per-axis gain (compact)
        sens = QtWidgets.QGroupBox("Sensitivity (raw/350 * gain)")
        sens_grid = QtWidgets.QGridLayout()

        sens_grid.addWidget(QtWidgets.QLabel("T:"), 0, 0)
        self.t_spin = QtWidgets.QDoubleSpinBox()
        self.t_spin.setRange(0.0001, 10.0); self.t_spin.setValue(0.05)
        self.t_spin.setSingleStep(0.005); self.t_spin.setDecimals(5)
        sens_grid.addWidget(self.t_spin, 0, 1)

        sens_grid.addWidget(QtWidgets.QLabel("R:"), 0, 2)
        self.r_spin = QtWidgets.QDoubleSpinBox()
        self.r_spin.setRange(0.0001, 10.0); self.r_spin.setValue(1.0)
        self.r_spin.setSingleStep(0.1); self.r_spin.setDecimals(5)
        sens_grid.addWidget(self.r_spin, 0, 3)

        t_labels = ["Tx", "Ty", "Tz"];  t_defaults = [1., 1., 1.]
        r_labels = ["Rx", "Ry", "Rz"];  r_defaults = [1., 1., -1.]
        self.gain_t, self.gain_r = [], []
        for col in range(3):
            lb = QtWidgets.QLabel(t_labels[col])
            lb.setStyleSheet("QLabel { font-size: 8pt; }")
            sens_grid.addWidget(lb, 1, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0); sp.setValue(t_defaults[col])
            sp.setSingleStep(0.1); sp.setDecimals(2)
            sp.setMaximumWidth(60)
            sens_grid.addWidget(sp, 2, col); self.gain_t.append(sp)
        for col in range(3):
            lb = QtWidgets.QLabel(r_labels[col])
            lb.setStyleSheet("QLabel { font-size: 8pt; }")
            sens_grid.addWidget(lb, 1, col+4)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0); sp.setValue(r_defaults[col])
            sp.setSingleStep(0.1); sp.setDecimals(2)
            sp.setMaximumWidth(60)
            sens_grid.addWidget(sp, 2, col+4); self.gain_r.append(sp)
        sens.setLayout(sens_grid)
        sens.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(sens)

        self.setLayout(layout)

    # -- Start/Stop ---------------------------------------------------

    def toggle(self):
        if self.active:
            self.timer.stop()
            self.reader.close()
            self.toggle_btn.setText("Start")
            self.active = False
            self.status_label.setText("Stopped")
        else:
            try:
                self.reader.connect()
                self.reader.start_reading(self._on_spacemouse_data)
                self.timer.start(4)  # Process at ~250Hz
                self.toggle_btn.setText("Stop")
                self.active = True
                self.status_label.setText("Running...")
            except RuntimeError as e:
                self.status_label.setText(f"Error: {e}")
                print(f"Failed to connect: {e}")

    # -- Driver toggle ------------------------------------------------

    def toggle_driver(self):
        new_state = not self.driver_enabled
        if self._write_driver_enabled(new_state):
            self.driver_enabled = new_state
            self._update_driver_btn()

    def toggle_space_mode(self):
        self.camera_space = not self.camera_space
        if self.camera_space:
            self.space_btn.setText("Cam Space")
            self.space_btn.setStyleSheet("QPushButton { background: #2a4a6a; color: #fff; }")
        else:
            self.space_btn.setText("Abs Space")
            self.space_btn.setStyleSheet("QPushButton { background: #6a2a2a; color: #fff; }")

    def _on_ch_toggle(self, idx):
        """Channel toggle: click=on/off, Alt+click=solo/unsolo"""
        modifiers = QtWidgets.QApplication.keyboardModifiers()
        alt_held = bool(modifiers & QtCore.Qt.AltModifier)

        if alt_held:
            if self.ch_solo == idx:
                # Unsolo: restore previous state
                self.ch_solo = None
                self.ch_active = list(self.ch_prev)
            else:
                # Solo this channel
                self.ch_prev = list(self.ch_active)
                self.ch_solo = idx
                self.ch_active = [i == idx for i in range(6)]
        else:
            # Simple toggle
            if self.ch_solo is not None:
                self.ch_solo = None
                self.ch_active = list(self.ch_prev)
            self.ch_active[idx] = not self.ch_active[idx]

        self._update_ch_buttons()

    def _ch_set_all(self, on):
        self.ch_solo = None
        self.ch_active = [on] * 6
        self._update_ch_buttons()

    def _ch_toggle_group(self, start):
        """Toggle all 3 channels in group (0=Txyz, 3=Rxyz)"""
        self.ch_solo = None
        all_on = all(self.ch_active[start:start+3])
        new_val = not all_on
        for i in range(start, start+3):
            self.ch_active[i] = new_val
        self._update_ch_buttons()

    def _update_ch_buttons(self):
        colors_on = ['#c44', '#4c4', '#44c', '#c44', '#4c4', '#44c']
        for i in range(6):
            if self.ch_active[i]:
                self.ch_btns[i].setStyleSheet(
                    f"QPushButton {{ background: {colors_on[i]}; color: #fff; font-size: 9pt; }}")
            else:
                self.ch_btns[i].setStyleSheet(
                    "QPushButton { background: #444; color: #888; font-size: 9pt; }")
        # T master: highlight if all 3 T channels on
        t_on = all(self.ch_active[0:3])
        self.t_master_btn.setStyleSheet(
            f"QPushButton {{ background: {'#0a0' if t_on else '#444'}; color: {'#fff' if t_on else '#888'}; font-size: 9pt; }}")
        # R master: highlight if all 3 R channels on
        r_on = all(self.ch_active[3:6])
        self.r_master_btn.setStyleSheet(
            f"QPushButton {{ background: {'#0a0' if r_on else '#444'}; color: {'#fff' if r_on else '#888'}; font-size: 9pt; }}")

    def _update_driver_btn(self):
        if self.driver_enabled:
            self.driver_btn.setText("Driver: ON (Camera)")
            self.driver_btn.setStyleSheet("QPushButton { background: #2a622a; color: #fff; }")
        else:
            self.driver_btn.setText("Driver: OFF (Object)")
            self.driver_btn.setStyleSheet("QPushButton { background: #6a4a1a; color: #fff; }")

    def _write_driver_enabled(self, enable):
        try:
            if not os.path.exists(DRIVER_CFG):
                return False
            tree = ET.parse(DRIVER_CFG)
            changed = 0
            for axis in tree.getroot().iter('Axis'):
                en = axis.find('Enabled')
                if en is not None:
                    new_val = 'true' if enable else 'false'
                    if en.text != new_val:
                        en.text = new_val; changed += 1
            if changed > 0:
                tree.write(DRIVER_CFG, encoding='UTF-8', xml_declaration=True)
            return True
        except Exception:
            return False

    # -- Detect parms -------------------------------------------------

    def print_scoped_parms(self):
        info = ""
        try:
            import apex
            sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
            kwargs = {}
            sv.runStateCommand('getState', kwargs)
            state = kwargs.get('state')
            if state:
                ctrls = getattr(state, 'control_selection', None)
                if ctrls:
                    info = f"Controls: {ctrls}\n"
                    # show current graph_parms values
                    from apex.control_2 import controlRigPath
                    cm = state.control_manager
                    scene = state.scene
                    for cp in ctrls:
                        rp = controlRigPath(cp)
                        rig = scene.getData(rp)
                        if rig and cm:
                            cpm = cm.getControlMapping(cp)
                            if cpm.t:
                                info += f"  {cpm.t}: {rig.graph_parms.get(cpm.t)}\n"
                            if cpm.r:
                                info += f"  {cpm.r}: {rig.graph_parms.get(cpm.r)}\n"
                else:
                    info = "No APEX controls selected"
            else:
                info = "No APEX state"
        except Exception as e:
            info = f"Error: {e}"
        print(info)
        self.status_label.setText(info)

    # -- Viewport helpers ---------------------------------------------

    def _get_viewport(self):
        try:
            desktop = hou.ui.curDesktop()
            viewer = desktop.paneTabOfType(hou.paneTabType.SceneViewer)
            return viewer.curViewport() if viewer else None
        except Exception:
            return None

    # -- Process data from HID thread ---------------------------------

    def process_spacemouse_data(self):
        """Called by Qt timer to process current SpaceMouse state"""
        try:
            # Get current data from HID thread
            with self.data_lock:
                tx, ty, tz = self.current_translation
                rx, ry, rz = self.current_rotation

            t_sens = self.t_spin.value()
            r_sens = self.r_spin.value()
            gt = np.array([g.value() for g in self.gain_t], dtype=np.float64)
            gr = np.array([g.value() for g in self.gain_r], dtype=np.float64)
            tv = np.array([tx, ty, tz], dtype=np.float64) / 350.0 * t_sens * gt
            rv = np.array([rx, ry, rz], dtype=np.float64) / 350.0 * r_sens * gr

            # Apply channel mask with corrected mapping:
            #   Button: Tx Ty Tz Rx Ry Rz
            #   Maps:  tv[0] tv[2] tv[1] rv[0] rv[2] rv[1]
            cm = [1.0 if a else 0.0 for a in self.ch_active]
            tv = tv * np.array([cm[0], cm[2], cm[1]], dtype=np.float64)
            rv = rv * np.array([cm[3], cm[5], cm[4]], dtype=np.float64)

            # Pre-mapped Houdini world coords
            hx, hy, hz = tv[0], -tv[2], tv[1]
            hrx, hry, hrz = rv[0], -rv[2], rv[1]

            if not self.driver_enabled:
                target = self._get_movable_target()
                if target is not None:
                    # Compute camera-relative deltas
                    use_cam = self.camera_space
                    if target[0] == 'apex':
                        use_cam = False  # APEX forces abs mode
                    cam_t = None
                    if use_cam:
                        viewport = self._get_viewport()
                        if viewport:
                            cm = mat4_to_numpy(viewport.viewTransform())
                            cr, cu, cf = cm[0,:3], cm[1,:3], cm[2,:3]
                            # Raw values as camera-relative
                            rtx, rty, rtz = tv[0], tv[1], tv[2]
                            cam_world = rtx*cr - rtz*cu + rty*cf
                            cam_t = (cam_world[0], cam_world[1], cam_world[2])
                    self._move_target(target, hx, hy, hz, hrx, hry, hrz, cam_t=cam_t)

            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)

        except Exception:
            import traceback
            traceback.print_exc()
            self.status_label.setText("Error: see console")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        viewport = self._get_viewport()
        cam_label = f"[Cam]" if viewport and viewport.camera() else "[No Cam]"
        mode = "[Object]" if not self.driver_enabled else "[Camera]"
        sp_mode = "Cam" if self.camera_space else "Abs"
        hx, hy, hz = tv[0], -tv[2], tv[1]
        info = (f"{mode} [{sp_mode}] | {cam_label}\n"
                f"raw T:({tx:+4d},{ty:+4d},{tz:+4d}) R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
                f"Hou  X:{hx:+.5f} Y:{hy:+.5f} Z:{hz:+.5f} R:({rv[0]:+.4f},{rv[1]:+.4f},{rv[2]:+.4f})")
        self.status_label.setText(info)

    # -- Object manipulation ------------------------------------------

    def _get_apex_state(self):
        try:
            import apex
            sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
            if sv is None: return None
            kwargs = {}
            sv.runStateCommand('getState', kwargs)
            return kwargs.get('state')
        except Exception:
            return None

    _last_target_kind = None
    _apex_skel_cache = {}  # {ctrl_name: world_matrix_numpy}

    def _get_movable_target(self):
        try:
            sel = hou.selectedNodes()
            if sel:
                node = sel[0]
                if hasattr(node, 'worldTransform') and callable(getattr(node, 'worldTransform', None)):
                    return ('obj', node)
                scoped = [p for p in node.parms() if p.isScoped()]
                if scoped: return ('ch', scoped)
                t_tuple = node.parmTuple('t')
                if t_tuple is not None and len(t_tuple) >= 3: return ('parm', node)
        except Exception: pass

        try:
            state = self._get_apex_state()
            if state is not None:
                ctrls = getattr(state, 'control_selection', None)
                if ctrls:
                    ctrl_list = list(ctrls)
                    # Build skeleton world-transform lookup
                    skel_lookup = self._apex_build_skel_lookup(state, ctrl_list)
                    return ('apex', (state, ctrl_list, skel_lookup))
        except Exception: pass

        self._last_target_kind = None
        return None

    _apex_dump_done = False

    def _apex_build_skel_lookup(self, state, ctrl_paths):
        """Build {control_name: world_matrix} from evaluated graph output"""
        lookup = {}
        try:
            from apex.control_2 import controlRigPath
            scene = state.scene
            if not ctrl_paths: return lookup
            rig_path = controlRigPath(ctrl_paths[0])
            rig = scene.getData(rig_path)
            if rig is None: return lookup
            g = rig.graph

            # Check all named output ports for geometry with named points + xform
            out_ports = g.outputPorts()
            for op in out_ports:
                try:
                    pn = op.portName()
                    data = g.getOutputData(pn)
                except Exception:
                    continue
                if data is None: continue

                # Geometry output
                if isinstance(data, hou.Geometry):
                    npts = data.intrinsicValue('pointcount')
                    if npts == 0: continue
                    pa = [a.name() for a in data.pointAttribs()]
                    if not SpaceMouseReceiver._apex_dump_done:
                        print(f"[SpaceMouse] {pn}: {npts}pts attrs={pa}")
                    has_name = 'name' in pa
                    has_xf = any(a in pa for a in ('xform', 'transform'))
                    if has_name and has_xf:
                        for pt in data.points():
                            try:
                                nm = pt.attribValue('name')
                                if not nm: continue
                                for an in ('xform', 'transform'):
                                    try:
                                        xf = pt.attribValue(an)
                                        if xf is not None and hasattr(xf, '__len__') and len(xf) in (12, 16):
                                            lookup[nm] = tuple_to_mat4(xf)
                                            break
                                    except: pass
                            except: pass

                # Dict output (from APEX Script dict::Build)
                elif isinstance(data, dict) or hasattr(data, 'keys'):
                    if not SpaceMouseReceiver._apex_dump_done:
                        print(f"[SpaceMouse] {pn}: dict keys={list(data.keys())[:10]}")
                    for k, v in data.items():
                        try:
                            if isinstance(v, hou.Matrix4):
                                lookup[k] = mat4_to_numpy(v)
                        except: pass

            SpaceMouseReceiver._apex_dump_done = True
        except Exception:
            pass
        return lookup

    def _log_target(self, kind, detail):
        if self._last_target_kind != kind:
            print(f"[SpaceMouse] target: [{kind}] {detail}")
            self._last_target_kind = kind

    def _move_target(self, target, tx, ty, tz, rx, ry, rz, raw_t=None, cam_t=None):
        kind = target[0]
        if kind == 'ch':
            self._move_ch_parms(target[1], tx, ty, tz, rx, ry, rz, cam_t=cam_t)
        elif kind == 'parm':
            self._move_parm_node(target[1], tx, ty, tz, rx, ry, rz, cam_t=cam_t)
        elif kind == 'apex':
            self._move_apex_controls(target[1], tx, ty, tz, rx, ry, rz, raw_t)
        elif kind == 'obj':
            self._move_obj_node(target[1], tx, ty, tz, rx, ry, rz, cam_t=cam_t)

    # Regex patterns for t{n}x / r{n}x
    _RE_TX = re.compile(r'(?:^|[_:])t\d*x$')
    _RE_TY = re.compile(r'(?:^|[_:])t\d*y$')
    _RE_TZ = re.compile(r'(?:^|[_:])t\d*z$')
    _RE_RX = re.compile(r'(?:^|[_:])r\d*x$')
    _RE_RY = re.compile(r'(?:^|[_:])r\d*y$')
    _RE_RZ = re.compile(r'(?:^|[_:])r\d*z$')
    _ch_printed = False

    def _move_ch_parms(self, parms, tx, ty, tz, rx, ry, rz, cam_t=None):
        t_map = [None, None, None]
        r_map = [None, None, None]
        is_rigpose = False
        pt_idx = 0
        for p in parms:
            name = p.name().lower()
            if SpaceMouseReceiver._RE_TX.search(name): t_map[0] = p
            elif SpaceMouseReceiver._RE_TY.search(name): t_map[1] = p
            elif SpaceMouseReceiver._RE_TZ.search(name): t_map[2] = p
            if SpaceMouseReceiver._RE_RX.search(name): r_map[0] = p
            elif SpaceMouseReceiver._RE_RY.search(name): r_map[1] = p
            elif SpaceMouseReceiver._RE_RZ.search(name): r_map[2] = p
            # Detect Rig Pose: digit in name → extract index
            m = re.search(r'(\d+)', name)
            if m and not is_rigpose:
                is_rigpose = True
                pt_idx = int(m.group(1))

        # Camera-relative: convert through bone world orientation
        if cam_t is not None and is_rigpose and all(t_map):
            world_rot = self._get_rigpose_world_rot(t_map[0], pt_idx)
            wd = np.array(cam_t)
            local_delta = wd @ world_rot.T
            tx, ty, tz = local_delta[0], local_delta[1], local_delta[2]
        elif cam_t is not None:
            tx, ty, tz = cam_t[0], cam_t[1], cam_t[2]

        if not SpaceMouseReceiver._ch_printed:
            print(f"[SpaceMouse] T:{t_map[0].name() if t_map[0] else '-'} "
                  f"{t_map[1].name() if t_map[1] else '-'} "
                  f"{t_map[2].name() if t_map[2] else '-'} | "
                  f"R:{r_map[0].name() if r_map[0] else '-'} "
                  f"{r_map[1].name() if r_map[1] else '-'} "
                  f"{r_map[2].name() if r_map[2] else '-'}"
                  f"  {'[rigpose]' if is_rigpose else ''}")
            SpaceMouseReceiver._ch_printed = True
        if all(t_map):
            t_map[0].set(t_map[0].eval() + tx)
            t_map[1].set(t_map[1].eval() + ty)
            t_map[2].set(t_map[2].eval() + tz)
        if all(r_map):
            rn_name = r_map[1].name() if r_map[1] else ''
            ry_val = -ry if re.search(r'\d', rn_name) else ry
            if abs(rx) > 1e-10: r_map[0].set(r_map[0].eval() + rx)
            if abs(ry) > 1e-10: r_map[1].set(r_map[1].eval() + ry_val)
            if abs(rz) > 1e-10: r_map[2].set(r_map[2].eval() + rz)

    def _get_rigpose_world_rot(self, parm, pt_idx):
        """Read world rotation of Rig Pose bone point from geometry output"""
        try:
            node = parm.node()
            geo = node.geometry() if hasattr(node, 'geometry') else None
            if geo is None and hasattr(node, 'displayNode'):
                dn = node.displayNode()
                geo = dn.geometry() if dn else None
            if geo is None: return np.eye(3)
            pts = list(geo.points())
            if pt_idx >= len(pts): return np.eye(3)
            pt = pts[pt_idx]
            # Build world matrix from P + transform(3x3)
            pos = np.array(pt.position(), dtype=np.float64)
            try:
                xf = pt.attribValue('transform')
                if xf is not None and hasattr(xf, '__len__') and len(xf) == 9:
                    rot33 = np.array(xf, dtype=np.float64).reshape(3, 3)
                else:
                    rot33 = np.eye(3)
            except Exception:
                rot33 = np.eye(3)
            return rot33
        except Exception:
            return np.eye(3)

    _apex_printed = False

    def _move_apex_controls(self, target, tx, ty, tz, rx, ry, rz, raw_t=None):
        """APEX: camera-relative movement via skeleton world transform"""
        state, ctrl_paths, skel_lookup = target
        if not ctrl_paths: return

        try:
            from apex.control_2 import controlRigPath
            ctrl_mgr = state.control_manager
            scene = state.scene

            viewport = self._get_viewport()
            cam_mat = mat4_to_numpy(viewport.viewTransform()) if viewport else np.eye(4)
            cam_rot = cam_mat[:3, :3]  # rows: right(0), up(1), fwd(2)

            # Camera-relative delta from RAW SpaceMouse values
            #   Tx+ = right→+cam_right   Ty- = fwd→-cam_fwd   Tz+ = down→-cam_up
            if raw_t is not None:
                rtx, rty, rtz = raw_t
                world_delta = -rtx*cam_rot[0,:] + rtz*cam_rot[1,:] + rty*cam_rot[2,:]
            else:
                world_delta = tx * cam_rot[0, :] + ty * cam_rot[1, :] + tz * cam_rot[2, :]

            for ctrl_path in ctrl_paths:
                rig_path = controlRigPath(ctrl_path)
                rig = scene.getData(rig_path)
                if rig is None: continue
                ctrl_map = ctrl_mgr.getControlMapping(ctrl_path)

                ctrl_name = ctrl_path.rsplit('/', 1)[-1] if '/' in ctrl_path else ctrl_path

                world_mat = skel_lookup.get(ctrl_name)
                world_rot = world_mat[:3, :3] if world_mat is not None else np.eye(3)

                local_delta = world_delta @ world_rot.T

                # Current local values
                cur_t = hou.Vector3(0, 0, 0)
                cur_r = hou.Vector3(0, 0, 0)
                if ctrl_map.t:
                    v = rig.graph_parms.get(ctrl_map.t)
                    cur_t = hou.Vector3(v) if v is not None and not isinstance(v, hou.Vector3) else (hou.Vector3(0,0,0) if v is None else hou.Vector3(v))
                if ctrl_map.r:
                    v = rig.graph_parms.get(ctrl_map.r)
                    cur_r = hou.Vector3(v) if v is not None and not isinstance(v, hou.Vector3) else (hou.Vector3(0,0,0) if v is None else hou.Vector3(v))

                if not SpaceMouseReceiver._apex_printed:
                    has_skel = "skel" if world_mat is not None else "world"
                    print(f"[SpaceMouse] APEX ctrl={ctrl_name} [{has_skel}] T={cur_t} R={cur_r}")
                    SpaceMouseReceiver._apex_printed = True

                if ctrl_map.t and (abs(tx) > 1e-10 or abs(ty) > 1e-10 or abs(tz) > 1e-10):
                    rig.graph_parms[ctrl_map.t] = hou.Vector3(
                        cur_t[0] + local_delta[0],
                        cur_t[1] + local_delta[1],
                        cur_t[2] + local_delta[2])

                if ctrl_map.r and (abs(rx) > 1e-10 or abs(ry) > 1e-10 or abs(rz) > 1e-10):
                    rig.graph_parms[ctrl_map.r] = hou.Vector3(
                        cur_r[0] + rx, cur_r[1] + ry, cur_r[2] + rz)

            state.runSceneCallbacks()

        except Exception:
            import traceback
            traceback.print_exc()

    def _move_parm_node(self, node, tx, ty, tz, rx, ry, rz, cam_t=None):
        if cam_t is not None: tx, ty, tz = cam_t[0], cam_t[1], cam_t[2]
        t_tuple = node.parmTuple('t')
        if t_tuple and len(t_tuple) >= 3:
            vals = t_tuple.eval(); t_tuple.set((vals[0]+tx, vals[1]+ty, vals[2]+tz))
        r_tuple = node.parmTuple('r')
        if r_tuple and len(r_tuple) >= 3:
            vals = r_tuple.eval()
            nrx, nry, nrz = vals[0], vals[1], vals[2]
            if abs(rx)>1e-10: nrx+=rx
            if abs(ry)>1e-10: nry+=ry
            if abs(rz)>1e-10: nrz+=rz
            r_tuple.set((nrx, nry, nrz))

    def _move_obj_node(self, node, tx, ty, tz, rx, ry, rz, cam_t=None):
        if cam_t is not None: tx, ty, tz = cam_t[0], cam_t[1], cam_t[2]
        obj_mat = mat4_to_numpy(node.worldTransform())
        obj_pos = obj_mat[3, :3].copy()
        obj_rot = obj_mat[:3, :3].copy()
        obj_pos += tx*obj_rot[0,:] + ty*obj_rot[1,:] + tz*obj_rot[2,:]
        if abs(ry) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0.,1.,0.]), np.radians(ry)).T
        if abs(rx) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([1.,0.,0.]), np.radians(rx)).T
        if abs(rz) > 1e-10:
            obj_rot = obj_rot @ rodrigues(np.array([0.,0.,1.]), np.radians(rz)).T
        obj_rot = orthonormalize(obj_rot)
        node.setWorldTransform(numpy_to_mat4(obj_pos, obj_rot))


def createInterface():
    return SpaceMouseReceiver()
