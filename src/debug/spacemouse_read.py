"""
SpaceMouse HID diagnostic tool -- SpaceExplorer
================================================
Reads raw HID reports from SpaceExplorer (VID:0x046D PID:0xC627)
and dumps hex data + parsed values to console.

Usage:
    python spacemouse_read.py

Independent script -- no Houdini dependency.
Requires the 'hid' module (hidapi).
"""
import sys
import os

# Allow import from parent src/ directory
_src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

import struct
import time

import config

try:
    import hid
except ImportError:
    print("ERROR: 'hid' module not installed.")
    print("Install with: pip install hidapi")
    sys.exit(1)

print("Searching for SpaceExplorer (VID=0x{:04X}, PID=0x{:04X})...".format(
    config.VID_3DCONNEXION, config.PID_SPACEEXPLORER))

# List all 3Dconnexion devices
devices = [d for d in hid.enumerate()
           if d['vendor_id'] == config.VID_3DCONNEXION]
if not devices:
    print("ERROR: No 3Dconnexion devices found")
    print("\nCheck:")
    print("  1. Device is plugged in")
    print("  2. Driver is installed")
    print("  3. Device Manager shows '3Dconnexion SpaceExplorer'")
    sys.exit(1)

print("\nFound {} 3Dconnexion device(s):".format(len(devices)))
for d in devices:
    print("  - PID=0x{:04X}, Usage Page={}, Usage={}".format(
        d['product_id'], d.get('usage_page', '?'), d.get('usage', '?')))
    print("    Path: {}".format(d['path']))

# Find SpaceExplorer (Usage Page=1, Usage=8)
target = None
for d in devices:
    if d['product_id'] == config.PID_SPACEEXPLORER:
        if d.get('usage_page') == config.HID_USAGE_PAGE and d.get('usage') == config.HID_USAGE:
            target = d
            break
        if target is None:
            target = d

if target is None:
    print("\nERROR: SpaceExplorer (PID=0x{:04X}) not found".format(
        config.PID_SPACEEXPLORER))
    sys.exit(1)

print("\nOpening: PID=0x{:04X}".format(target['product_id']))

dev = hid.device()
try:
    dev.open_path(target['path'])
except IOError as e:
    print("\nOpen failed: {}".format(e))
    print("\nPossible causes:")
    print("  1. 3DxWare service holds the device exclusively")
    print("     Fix: run 'net stop 3DxService' as admin")
    print("  2. Insufficient permissions")
    print("     Fix: run this script as admin")
    sys.exit(1)

print("Connected!")
print("Manufacturer: {}".format(dev.get_manufacturer_string()))
print("Product: {}".format(dev.get_product_string()))
print("\nMove the SpaceExplorer to see data (Ctrl+C to exit)\n")
print("{:>6} {:>50}".format("ReportID", "Raw Data (hex)"))
print("-" * 60)

try:
    count = 0
    while True:
        data = dev.read(64, timeout_ms=100)
        if data:
            count += 1
            hex_str = ' '.join('{:02X}'.format(b) for b in data[:13])
            print("{:>6}  {}".format(count, hex_str))

            # Report 1: Translation (Tx, Ty, Tz) — CORRECT mapping
            if data[0] == config.REPORT_ID_TRANSLATION and len(data) >= 7:
                tx = struct.unpack('<h', bytes(data[1:3]))[0]
                ty = struct.unpack('<h', bytes(data[3:5]))[0]
                tz = struct.unpack('<h', bytes(data[5:7]))[0]
                print("       -> Translation: X={:+5d}  Y={:+5d}  Z={:+5d}".format(tx, ty, tz))

            # Report 2: Rotation (Rx, Ry, Rz) — CORRECT mapping
            elif data[0] == config.REPORT_ID_ROTATION and len(data) >= 7:
                rx = struct.unpack('<h', bytes(data[1:3]))[0]
                ry = struct.unpack('<h', bytes(data[3:5]))[0]
                rz = struct.unpack('<h', bytes(data[5:7]))[0]
                print("       -> Rotation: RX={:+5d}  RY={:+5d}  RZ={:+5d}".format(rx, ry, rz))

            # Report 3: Buttons — CORRECT mapping
            elif data[0] == config.REPORT_ID_BUTTONS and len(data) >= 4:
                btn = (data[1] | (data[2] << 8) | (data[3] << 16))
                if btn:
                    print("       -> Buttons: 0x{:08X}".format(btn))

except KeyboardInterrupt:
    print("\n\nStopped")
finally:
    dev.close()
