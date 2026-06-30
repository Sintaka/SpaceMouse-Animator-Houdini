"""
SpaceMouse Controller — orchestrator for the Houdini Python Panel.

Wires together:
  - HID reader (daemon thread → queue.Queue)
  - Data processor (raw → Houdini world coordinates)
  - Target detector (find movable target)
  - Target mover (apply movement)
  - Driver control (3DxWare toggle)
  - UI panel (Qt widgets)
  - Debug logging (tee-print)

Provides create_interface() for the Houdini Python Panel entry point.
"""
import queue
import numpy as np

from PySide6 import QtCore

import config
import debug_utils
import hid_reader
import data_processor
import driver_control
import target_detector
import target_mover
import apex_utils
import matrix_utils
from ui_panel import SpaceMousePanelUI


class SpaceMouseController(QtCore.QObject):
    """Orchestrates HID reading, data processing, target manipulation, and UI."""

    def __init__(self):
        super().__init__()

        # --- Debug logging ---
        self._log_path = debug_utils.setup_debug_logging()
        print("[SpaceMouse] Debug log: {}".format(self._log_path))

        # --- Data queue (thread-safe bridge between HID thread and UI timer) ---
        self._data_queue = queue.Queue(maxsize=config.DATA_QUEUE_SIZE)

        # --- HID reader ---
        self._hid_reader = hid_reader.HIDReader(self._data_queue)

        # --- UI ---
        self.ui = SpaceMousePanelUI()

        # --- Timer ---
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._on_timer_tick)

        # --- State ---
        self._active = False
        self._driver_enabled = True
        self._camera_space = True

        # --- Wire UI signals ---
        self.ui.toggle_btn.clicked.connect(self._on_toggle)
        self.ui.detect_btn.clicked.connect(self._on_detect)
        self.ui.driver_btn.clicked.connect(self._on_driver_toggle)
        self.ui.space_btn.clicked.connect(self._on_space_toggle)

        # --- Prevent GC: controller must outlive the UI widget ---
        self.ui._controller = self

        # --- Cleanup ---
        self.ui.destroyed.connect(self._cleanup)

        # --- Initial driver state ---
        driver_control.set_driver_enabled(True)
        self._driver_enabled = True
        self.ui.set_driver_button_state(True)

    # ==================================================================
    # Public API
    # ==================================================================

    def start(self):
        """Connect HID device and start polling."""
        try:
            self._hid_reader.connect()
            self._hid_reader.start()
            self._timer.start(config.TIMER_INTERVAL_MS)
            self._active = True
            self.ui.set_toggle_button_state(True)
            self.ui.status_label.setText("Running...")
            print("[SpaceMouse] Started")
        except Exception as e:
            self.ui.status_label.setText("Error: {}".format(e))
            print("[SpaceMouse] Start error: {}".format(e))

    def stop(self):
        """Stop polling and disconnect HID device."""
        self._timer.stop()
        self._hid_reader.stop()
        self._hid_reader.close()
        self._active = False
        target_detector.reset_target_log()
        target_mover.reset_move_state()
        self.ui.set_toggle_button_state(False)
        self.ui.status_label.setText("Stopped")
        print("[SpaceMouse] Stopped")

    # ==================================================================
    # Timer callback — main processing loop
    # ==================================================================

    def _on_timer_tick(self):
        """Called at ~250 Hz. Drains HID data queue and processes frames."""
        try:
            # Drain all available frames; only process the latest one
            raw_data = None
            drained = 0
            while True:
                try:
                    raw_data = self._data_queue.get_nowait()
                    drained += 1
                except queue.Empty:
                    break

            if raw_data is None:
                return

            # Read current sensitivity / gain / mask from UI widgets
            t_sens = self.ui.t_spin.value()
            r_sens = self.ui.r_spin.value()
            t_gains = [g.value() for g in self.ui.gain_t]
            r_gains = [g.value() for g in self.ui.gain_r]
            ch_mask = self.ui.ch_active

            # Process raw data → Houdini world coordinates
            hx, hy, hz, hrx, hry, hrz = data_processor.process_frame(
                raw_data, t_sens, r_sens, t_gains, r_gains, ch_mask)

            # Update display label
            self._update_display(raw_data, hx, hy, hz, hrx, hry, hrz)

            # If driver is OFF, move the selected target
            if not self._driver_enabled:
                target = target_detector.detect_target()
                if target is not None:
                    # Compute camera-relative deltas if needed
                    use_cam = self._camera_space

                    cam_t = None
                    raw_t = None
                    if use_cam:
                        viewport = target_mover._get_viewport()
                        if viewport:
                            # Compute raw TV with channel mask applied
                            tx, ty, tz = raw_data['translation']
                            gt = np.array(t_gains, dtype=np.float64)
                            tv = (np.array([tx, ty, tz], dtype=np.float64) /
                                  config.AXIS_RANGE * t_sens * gt)
                            # Apply T channel mask (same swap as process_frame)
                            cm = [1.0 if a else 0.0 for a in ch_mask]
                            t_mask = np.array([cm[i] for i in config.CH_MASK_T_MAP],
                                              dtype=np.float64)
                            tv = tv * t_mask
                            cam_t = data_processor.compute_camera_delta(
                                tv, matrix_utils.mat4_to_numpy(viewport.viewTransform()))
                            raw_t = (tv[0], tv[1], tv[2])

                    # Log target kind
                    kind = target[0]
                    detail = _target_detail(kind, target[1])
                    target_detector.log_target(kind, detail)

                    # Apply movement
                    target_mover.move_target(
                        target, hx, hy, hz, hrx, hry, hrz,
                        raw_t=raw_t, cam_t=cam_t, use_cam=use_cam)

        except Exception:
            import traceback
            traceback.print_exc()
            self.ui.status_label.setText("Error: see console")

    # ==================================================================
    # Display
    # ==================================================================

    def _update_display(self, raw_data, hx, hy, hz, hrx, hry, hrz):
        """Refresh the status label with current axis values."""
        tx, ty, tz = raw_data['translation']
        rx, ry, rz = raw_data['rotation']

        viewport = target_mover._get_viewport()
        cam_label = "[Cam]" if viewport and viewport.camera() else "[No Cam]"
        mode = "[Camera]" if self._driver_enabled else "[Object]"
        sp_mode = "Cam" if self._camera_space else "Abs"

        info = ("{} [{}] | {}\n"
                "raw T:({:+4d},{:+4d},{:+4d}) R:({:+4d},{:+4d},{:+4d})\n"
                "Hou  X:{:+.5f} Y:{:+.5f} Z:{:+.5f}"
                " R:({:+.4f},{:+.4f},{:+.4f})").format(
            mode, sp_mode, cam_label,
            tx, ty, tz, rx, ry, rz,
            hx, hy, hz, hrx, hry, hrz)
        self.ui.status_label.setText(info)

    # ==================================================================
    # Button slots
    # ==================================================================

    def _on_toggle(self):
        """Start/Stop button."""
        if self._active:
            self.stop()
        else:
            self.start()

    def _on_detect(self):
        """Detect button — show APEX control or scoped parm info."""
        info = ""
        try:
            state = apex_utils.get_apex_state()
            if state:
                info = apex_utils.get_scoped_controls_info(state)
            else:
                info = "No APEX state"
        except Exception as e:
            info = "Error: {}".format(e)
        print(info)
        self.ui.status_label.setText(info)

    def _on_driver_toggle(self):
        """Toggle 3DxWare driver on/off."""
        new_state = not self._driver_enabled
        if driver_control.set_driver_enabled(new_state):
            self._driver_enabled = new_state
            self.ui.set_driver_button_state(new_state)

    def _on_space_toggle(self):
        """Toggle camera-relative / absolute world space."""
        self._camera_space = not self._camera_space
        self.ui.set_space_button_state(self._camera_space)

    # ==================================================================
    # Cleanup
    # ==================================================================

    def _cleanup(self):
        """Release resources on panel close."""
        if self._active:
            self.stop()
        print("[SpaceMouse] Panel closed")


def _target_detail(kind, data):
    """Return a short human-readable description of the current target."""
    if kind == 'obj':
        return data.name() if hasattr(data, 'name') else 'obj'
    elif kind == 'parm':
        return data.name() if hasattr(data, 'name') else 'parm'
    elif kind == 'ch':
        return "{} scoped parms".format(len(data))
    elif kind == 'apex':
        return "{} controls".format(len(data[1]))
    return "unknown"


def create_interface():
    """Create the SpaceMouse panel widget for Houdini Python Panel.

    This is the standard entry point called by Houdini's Python Panel
    interface. It creates a SpaceMouseController and returns its UI widget.

    Returns:
        SpaceMousePanelUI (QWidget).
    """
    controller = SpaceMouseController()
    return controller.ui
