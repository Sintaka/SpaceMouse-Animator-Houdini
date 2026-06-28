"""
SpaceMouse UDP broadcaster -- SpaceExplorer
Reads SpaceExplorer (VID:046D PID:C627) raw HID data, broadcasts via UDP

Axis mapping (SpaceExplorer tested):
  translation[0]=Tx: left(-)/right(+)   rotation[0]=Rx: Pitch  fwd(-)/back(+)
  translation[1]=Ty: fwd(-)/back(+)     rotation[1]=Ry: Roll   cw(-)/ccw(+)
  translation[2]=Tz: down(+)/up(-)      rotation[2]=Rz: Yaw    cw(-)/ccw(+) (top view)

Report rate: ~125 pkt/s (T/R alternating), full 6DOF ~62.5 Hz
"""
import hid
import struct
import socket
import json
import time
import threading

VENDOR_3DCONNEXION = 0x046D
PRODUCT_SPACEEXPLORER = 0xC627
UDP_HOST = "127.0.0.1"
UDP_PORT = 9876


class SpaceMouseReader:
    def __init__(self, broadcaster):
        self.dev = None
        self.broadcaster = broadcaster
        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0]
        self.button_state = 0
        self.running = False
        self.frame_count = 0
        self.lock = threading.Lock()

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

    def start_reading(self):
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
                should_send = False

                with self.lock:
                    # Report 1: Translation (Tx, Ty, Tz)
                    if report_id == 1 and len(data) >= 7:
                        self.translation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.translation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.translation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True

                    # Report 2: Rotation (Rx, Ry, Rz)
                    elif report_id == 2 and len(data) >= 7:
                        self.rotation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.rotation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.rotation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True

                    # Report 3: Button state
                    elif report_id == 3 and len(data) >= 4:
                        self.button_state = (data[1] |
                                            (data[2] << 8) |
                                            (data[3] << 16))
                        should_send = True

                    if should_send:
                        packet = {
                            'timestamp': time.time(),
                            'frame': self.frame_count,
                            'translation': self.translation[:],
                            'rotation': self.rotation[:],
                            'buttons': self.button_state
                        }
                        self.broadcaster.send(packet)

                        tx, ty, tz = self.translation
                        rx, ry, rz = self.rotation
                        print(f"[{self.frame_count:6d}] "
                              f"Tx:{tx:+5d} Ty:{ty:+5d} Tz:{tz:+5d} | "
                              f"Rx:{rx:+5d} Ry:{ry:+5d} Rz:{rz:+5d} | "
                              f"BTN:{self.button_state:08X}")

                        self.frame_count += 1

            except Exception as e:
                if self.running:
                    print(f"Read error: {e}")
                    time.sleep(0.1)

    def close(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
        if self.dev:
            self.dev.close()


class UDPBroadcaster:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        print(f"UDP target: {host}:{port}")

    def send(self, data_dict):
        try:
            msg = json.dumps(data_dict).encode('utf-8')
            self.sock.sendto(msg, self.addr)
        except Exception as e:
            print(f"Send error: {e}")

    def close(self):
        self.sock.close()


def main():
    print("=" * 60)
    print("  SpaceMouse UDP Broadcaster")
    print("=" * 60)

    broadcaster = UDPBroadcaster(UDP_HOST, UDP_PORT)
    reader = SpaceMouseReader(broadcaster)
    reader.connect()
    reader.start_reading()

    print(f"\nBroadcasting (Ctrl+C to stop)...\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        reader.close()
        broadcaster.close()
        print("Done")


if __name__ == "__main__":
    main()
