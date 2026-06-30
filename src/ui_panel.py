"""
Pure Qt UI for the SpaceMouse Houdini Python Panel.
Constructs the widget hierarchy, layout, and signal declarations ONLY.
No business logic — no HID, no Houdini node manipulation, no movement math.

All widgets are exposed as public attributes so the controller can
read values and connect slots.
"""
from PySide6 import QtCore, QtWidgets


class SpaceMousePanelUI(QtWidgets.QWidget):
    """Qt widget tree for the SpaceMouse control panel.

    Public widget attributes (read by controller):
        status_label    — QLabel, monospace status display.
        toggle_btn      — QPushButton, Start/Stop.
        detect_btn      — QPushButton, detect APEX/scoped parms.
        driver_btn      — QPushButton, 3DxWare driver toggle.
        space_btn       — QPushButton, camera/absolute space toggle.
        ch_btns         — list of 6 QPushButton, channel toggles [Tx,Ty,Tz,Rx,Ry,Rz].
        ch_active       — list of 6 bool, current channel on/off state.
        ch_solo         — int or None, solo'd channel index.
        t_master_btn    — QPushButton, T group master toggle.
        r_master_btn    — QPushButton, R group master toggle.
        t_spin          — QDoubleSpinBox, translation sensitivity.
        r_spin          — QDoubleSpinBox, rotation sensitivity.
        gain_t          — list of 3 QDoubleSpinBox, per-axis T gains.
        gain_r          — list of 3 QDoubleSpinBox, per-axis R gains.
    """

    def __init__(self):
        super().__init__()

        # Channel state (owned by UI, read by controller)
        self.ch_active = [True] * 6
        self.ch_solo = None
        self.ch_prev = [True] * 6

        self._init_ui()

    # ==================================================================
    # UI construction
    # ==================================================================

    def _init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # -- Status label --
        self.status_label = QtWidgets.QLabel("Waiting for SpaceMouse data...")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(110)
        self.status_label.setStyleSheet(
            "QLabel { font-family: Consolas, monospace; font-size: 12px;"
            " background: #1e1e1e; color: #d4d4d4; padding: 8px;"
            " border: 1px solid #444; border-radius: 4px; }")
        layout.addWidget(self.status_label)

        # -- Start/Stop + Detect row --
        btn_row = QtWidgets.QHBoxLayout()
        self.toggle_btn = QtWidgets.QPushButton("Start")
        self.toggle_btn.setMinimumHeight(30)
        btn_row.addWidget(self.toggle_btn)

        self.detect_btn = QtWidgets.QPushButton("Detect Parms")
        self.detect_btn.setMinimumHeight(30)
        btn_row.addWidget(self.detect_btn)
        layout.addLayout(btn_row)

        # -- Driver toggle + Space mode row --
        drv_row = QtWidgets.QHBoxLayout()
        drv_row.addWidget(QtWidgets.QLabel("3DxWare:"))

        self.driver_btn = QtWidgets.QPushButton("Driver: ON (Camera)")
        self.driver_btn.setMinimumHeight(30)
        drv_row.addWidget(self.driver_btn)

        # Camera-space toggle
        self.space_btn = QtWidgets.QPushButton("Cam Space")
        self.space_btn.setMinimumHeight(30)
        self.space_btn.setMinimumWidth(72)
        self.space_btn.setStyleSheet(
            "QPushButton { background: #2a4a6a; color: #fff; }")
        self.space_btn.setToolTip(
            "Toggle movement space\n"
            "Cam = camera-relative\n"
            "Abs = world absolute")
        drv_row.addWidget(self.space_btn)
        layout.addLayout(drv_row)

        # -- Channel toggle buttons --
        ch_row = QtWidgets.QHBoxLayout()
        ch_row.addStretch()

        ch_labels = ['Tx', 'Ty', 'Tz', 'Rx', 'Ry', 'Rz']
        ch_colors = ['#c44', '#4c4', '#44c', '#c44', '#4c4', '#44c']
        self.ch_btns = []

        for i in range(6):
            btn = QtWidgets.QPushButton(ch_labels[i])
            btn.setMinimumHeight(22)
            btn.setMinimumWidth(40)
            btn.setMaximumWidth(46)
            btn.setStyleSheet(
                "QPushButton {{ background: {}; color: #fff;"
                " font-size: 9pt; padding: 1px 4px; }}".format(ch_colors[i]))
            btn.setToolTip(
                "{}\nClick: toggle on/off\nAlt+Click: solo/unsolo".format(ch_labels[i]))
            btn.clicked.connect(lambda _=None, idx=i: self._on_ch_toggle(idx))
            self.ch_btns.append(btn)
            ch_row.addWidget(btn)

        ch_row.addSpacing(4)

        # T / R master toggles
        self.t_master_btn = QtWidgets.QPushButton("T")
        self.t_master_btn.setMinimumHeight(22)
        self.t_master_btn.setMaximumWidth(28)
        self.t_master_btn.clicked.connect(lambda: self._ch_toggle_group(0))
        ch_row.addWidget(self.t_master_btn)

        self.r_master_btn = QtWidgets.QPushButton("R")
        self.r_master_btn.setMinimumHeight(22)
        self.r_master_btn.setMaximumWidth(28)
        self.r_master_btn.clicked.connect(lambda: self._ch_toggle_group(3))
        ch_row.addWidget(self.r_master_btn)

        # All / None
        for label, cb in [('All', lambda: self._ch_set_all(True)),
                           ('None', lambda: self._ch_set_all(False))]:
            btn = QtWidgets.QPushButton(label)
            btn.setMinimumHeight(22)
            btn.setMaximumWidth(40)
            btn.setStyleSheet(
                "QPushButton { background: #555; color: #ccc;"
                " font-size: 9pt; padding: 1px 4px; }")
            btn.clicked.connect(lambda _=None, f=cb: f())
            ch_row.addWidget(btn)

        self._update_ch_buttons()
        layout.addLayout(ch_row)

        # -- Sensitivity + Per-axis gain --
        sens = QtWidgets.QGroupBox("Sensitivity (raw/350 * gain)")
        sens_grid = QtWidgets.QGridLayout()

        sens_grid.addWidget(QtWidgets.QLabel("T:"), 0, 0)
        self.t_spin = QtWidgets.QDoubleSpinBox()
        self.t_spin.setRange(0.0001, 10.0)
        self.t_spin.setValue(0.05)
        self.t_spin.setSingleStep(0.005)
        self.t_spin.setDecimals(5)
        sens_grid.addWidget(self.t_spin, 0, 1)

        sens_grid.addWidget(QtWidgets.QLabel("R:"), 0, 2)
        self.r_spin = QtWidgets.QDoubleSpinBox()
        self.r_spin.setRange(0.0001, 10.0)
        self.r_spin.setValue(1.0)
        self.r_spin.setSingleStep(0.1)
        self.r_spin.setDecimals(5)
        sens_grid.addWidget(self.r_spin, 0, 3)

        t_labels = ["Tx", "Ty", "Tz"]
        t_defaults = [1.0, 1.0, 1.0]
        r_labels = ["Rx", "Ry", "Rz"]
        r_defaults = [1.0, 1.0, -1.0]

        self.gain_t = []
        self.gain_r = []

        for col in range(3):
            lb = QtWidgets.QLabel(t_labels[col])
            lb.setStyleSheet("QLabel { font-size: 8pt; }")
            sens_grid.addWidget(lb, 1, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(t_defaults[col])
            sp.setSingleStep(0.1)
            sp.setDecimals(2)
            sp.setMaximumWidth(60)
            sens_grid.addWidget(sp, 2, col)
            self.gain_t.append(sp)

        for col in range(3):
            lb = QtWidgets.QLabel(r_labels[col])
            lb.setStyleSheet("QLabel { font-size: 8pt; }")
            sens_grid.addWidget(lb, 1, col + 4)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(r_defaults[col])
            sp.setSingleStep(0.1)
            sp.setDecimals(2)
            sp.setMaximumWidth(60)
            sens_grid.addWidget(sp, 2, col + 4)
            self.gain_r.append(sp)

        sens.setLayout(sens_grid)
        sens.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                           QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(sens)

        self.setLayout(layout)

    # ==================================================================
    # Channel toggle logic (UI-only — reads/writes ch_active, ch_solo)
    # ==================================================================

    def _on_ch_toggle(self, idx):
        """Channel button: click = on/off, Alt+click = solo/unsolo."""
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
        """Set all 6 channels on or off."""
        self.ch_solo = None
        self.ch_active = [on] * 6
        self._update_ch_buttons()

    def _ch_toggle_group(self, start):
        """Toggle all 3 channels in a group (0=Txyz, 3=Rxyz)."""
        self.ch_solo = None
        all_on = all(self.ch_active[start:start + 3])
        new_val = not all_on
        for i in range(start, start + 3):
            self.ch_active[i] = new_val
        self._update_ch_buttons()

    def _update_ch_buttons(self):
        """Refresh button styling to reflect ch_active state."""
        colors_on = ['#c44', '#4c4', '#44c', '#c44', '#4c4', '#44c']
        for i in range(6):
            if self.ch_active[i]:
                self.ch_btns[i].setStyleSheet(
                    "QPushButton {{ background: {}; color: #fff;"
                    " font-size: 9pt; }}".format(colors_on[i]))
            else:
                self.ch_btns[i].setStyleSheet(
                    "QPushButton { background: #444; color: #888;"
                    " font-size: 9pt; }")

        # T master highlight
        t_on = all(self.ch_active[0:3])
        self.t_master_btn.setStyleSheet(
            "QPushButton {{ background: {}; color: {};"
            " font-size: 9pt; }}".format('#0a0' if t_on else '#444',
                                         '#fff' if t_on else '#888'))

        # R master highlight
        r_on = all(self.ch_active[3:6])
        self.r_master_btn.setStyleSheet(
            "QPushButton {{ background: {}; color: {};"
            " font-size: 9pt; }}".format('#0a0' if r_on else '#444',
                                         '#fff' if r_on else '#888'))

    # ==================================================================
    # Button styling helpers (called by controller)
    # ==================================================================

    def set_driver_button_state(self, enabled):
        """Update driver button text and style.

        Args:
            enabled: bool — True = driver ON (camera mode),
                              False = driver OFF (object mode).
        """
        if enabled:
            self.driver_btn.setText("Driver: ON (Camera)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #2a622a; color: #fff; }")
        else:
            self.driver_btn.setText("Driver: OFF (Object)")
            self.driver_btn.setStyleSheet(
                "QPushButton { background: #6a4a1a; color: #fff; }")

    def set_space_button_state(self, camera_space):
        """Update camera/absolute space button.

        Args:
            camera_space: bool — True = camera-relative, False = absolute world.
        """
        if camera_space:
            self.space_btn.setText("Cam Space")
            self.space_btn.setStyleSheet(
                "QPushButton { background: #2a4a6a; color: #fff; }")
        else:
            self.space_btn.setText("Abs Space")
            self.space_btn.setStyleSheet(
                "QPushButton { background: #6a2a2a; color: #fff; }")

    def set_toggle_button_state(self, active):
        """Update Start/Stop button text.

        Args:
            active: bool — True when running, False when stopped.
        """
        if active:
            self.toggle_btn.setText("Stop")
        else:
            self.toggle_btn.setText("Start")
