"""
SpaceMouse UDP broadcaster -- SpaceExplorer
===========================================
Standalone script for testing outside Houdini.
Reads SpaceExplorer (VID:046D PID:C627) raw HID data and broadcasts via UDP.

Usage:
    python spacemouse_sender.py

This is a standalone tool; it does NOT depend on Houdini.
Uses the 'hid' module for direct HID access and UDP for transport.
"""
import sys
import os

# Allow import from parent src/ directory
_src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

import struct
import socket
import json
import time
import threading

import config
from hid_reader import HIDReader

UDP_HOST = "127.0.0.1"
UDP_PORT = 9876


class UDPBroadcaster:
    """Sends JSON-encoded data dicts over UDP to localhost."""

    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        print("UDP target: {}:{}".format(host, port))

    def send(self, data_dict):
        try:
            msg = json.dumps(data_dict).encode('utf-8')
            self.sock.sendto(msg, self.addr)
        except Exception as e:
            print("Send error: {}".format(e))

    def close(self):
        self.sock.close()


def main():
    print("=" * 60)
    print("  SpaceMouse UDP Broadcaster (standalone)")
    print("=" * 60)

    broadcaster = UDPBroadcaster(UDP_HOST, UDP_PORT)

    # Use a simple list-as-queue for standalone mode
    # HIDReader pushes to a queue; we poll it here
    import queue
    data_queue = queue.Queue(maxsize=config.DATA_QUEUE_SIZE)

    reader = HIDReader(data_queue)
    reader.connect()
    reader.start()

    print("\nBroadcasting (Ctrl+C to stop)...\n")

    try:
        frame_count = 0
        while True:
            try:
                frame_data = data_queue.get(timeout=0.5)
                # Add UDP broadcast
                broadcaster.send(frame_data)

                tx, ty, tz = frame_data['translation']
                rx, ry, rz = frame_data['rotation']
                print("[{:6d}] Tx:{:+5d} Ty:{:+5d} Tz:{:+5d} | "
                      "Rx:{:+5d} Ry:{:+5d} Rz:{:+5d} | "
                      "BTN:{:08X}".format(
                          frame_data['frame'],
                          tx, ty, tz, rx, ry, rz,
                          frame_data['buttons']))
                frame_count += 1
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        reader.close()
        broadcaster.close()
        print("Done")


if __name__ == "__main__":
    main()
