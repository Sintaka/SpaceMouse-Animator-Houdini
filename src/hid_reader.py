"""
HID device reader for 3Dconnexion SpaceExplorer.
Reads raw HID reports in a daemon thread and pushes parsed data dicts
into a queue.Queue for consumption by the UI thread.

No UDP dependency — pure in-process HID access via the 'hid' module.
"""
import struct
import time
import threading
import queue

try:
    import hid
    _HID_AVAILABLE = True
except ImportError:
    _HID_AVAILABLE = False

import config


class HIDReader:
    """Reads SpaceExplorer HID reports in a background thread.

    Parsed frames are pushed into a thread-safe queue.Queue for the
    UI thread to consume via a Qt timer.
    """

    def __init__(self, data_queue):
        """Initialise the reader.

        Args:
            data_queue: queue.Queue instance where parsed data dicts are pushed.
        """
        self._queue = data_queue
        self._dev = None
        self._translation = [0, 0, 0]
        self._rotation = [0, 0, 0]
        self._button_state = 0
        self._frame_count = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._connected = False

    @property
    def connected(self):
        return self._connected

    def connect(self):
        """Enumerate HID devices and open the SpaceExplorer.

        Raises:
            RuntimeError: If no matching device is found.
        """
        if not _HID_AVAILABLE:
            raise RuntimeError("hid module not installed")

        devices = [d for d in hid.enumerate()
                   if d['vendor_id'] == config.VID_3DCONNEXION
                   and d['product_id'] == config.PID_SPACEEXPLORER
                   and d.get('usage_page') == config.HID_USAGE_PAGE
                   and d.get('usage') == config.HID_USAGE]

        if not devices:
            raise RuntimeError(
                f"SpaceExplorer (VID:0x{config.VID_3DCONNEXION:04X}, "
                f"PID:0x{config.PID_SPACEEXPLORER:04X}) not found. "
                f"Check USB connection and 3DxWare driver state."
            )

        self._dev = hid.device()
        self._dev.open_path(devices[0]['path'])
        self._dev.set_nonblocking(0)
        self._connected = True
        print(f"[HIDReader] Connected: {self._dev.get_product_string()}")

    def start(self):
        """Spawn the daemon read thread."""
        if not self._connected:
            raise RuntimeError("Device not connected. Call connect() first.")
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("[HIDReader] Read thread started")

    def stop(self):
        """Signal the read thread to stop."""
        self._running = False

    def close(self):
        """Stop the read thread and close the HID device."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._dev is not None:
            self._dev.close()
            self._dev = None
            self._connected = False
        print("[HIDReader] Device closed")

    def _read_loop(self):
        """Main loop running in the daemon thread."""
        while self._running:
            try:
                data = self._dev.read(64, timeout_ms=1)
                if not data:
                    continue

                report_id = data[0]
                should_send = False

                with self._lock:
                    # Report 1: Translation (Tx, Ty, Tz)
                    if report_id == config.REPORT_ID_TRANSLATION and len(data) >= 7:
                        self._translation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self._translation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self._translation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True

                    # Report 2: Rotation (Rx, Ry, Rz)
                    elif report_id == config.REPORT_ID_ROTATION and len(data) >= 7:
                        self._rotation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self._rotation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self._rotation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True

                    # Report 3: Button state
                    elif report_id == config.REPORT_ID_BUTTONS and len(data) >= 4:
                        self._button_state = (data[1] |
                                             (data[2] << 8) |
                                             (data[3] << 16))
                        should_send = True

                    if should_send:
                        frame_data = {
                            'timestamp': time.time(),
                            'frame': self._frame_count,
                            'translation': self._translation[:],
                            'rotation': self._rotation[:],
                            'buttons': self._button_state,
                        }
                        # Non-blocking put; drop oldest if queue is full
                        try:
                            self._queue.put_nowait(frame_data)
                        except queue.Full:
                            pass  # queue is full, skip this frame

                        self._frame_count += 1

            except Exception as e:
                if self._running:
                    print(f"[HIDReader] Read error: {e}")
                    time.sleep(0.1)
