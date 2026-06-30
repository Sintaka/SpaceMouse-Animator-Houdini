"""
SpaceMouse configuration constants.
Device: 3Dconnexion SpaceExplorer (VID:046D, PID:C627)
"""
import os


def get_src_dir():
    """Return the absolute path to the src/ directory.

    Uses __file__ of this module, so it works regardless of the
    current working directory or where Houdini was launched from.
    """
    return os.path.dirname(os.path.abspath(__file__))

# === HID Hardware IDs ===
VID_3DCONNEXION = 0x046D
PID_SPACEEXPLORER = 0xC627
HID_USAGE_PAGE = 1        # Generic Desktop
HID_USAGE = 8             # Multi-axis Controller

# === HID Report IDs ===
REPORT_ID_TRANSLATION = 1  # 0x01 — Tx, Ty, Tz (int16 LE)
REPORT_ID_ROTATION = 2     # 0x02 — Rx, Ry, Rz (int16 LE)
REPORT_ID_BUTTONS = 3      # 0x03 — button bitmask

# === Data processing ===
AXIS_RANGE = 350.0                           # max raw int16 range (-350..+350)

# === Sensitivity defaults ===
DEFAULT_T_SENSITIVITY = 0.05                 # translation sensitivity
DEFAULT_R_SENSITIVITY = 1.0                  # rotation sensitivity
DEFAULT_T_GAINS = [1.0, 1.0, 1.0]           # Tx, Ty, Tz per-axis gains
DEFAULT_R_GAINS = [1.0, 1.0, -1.0]          # Rx, Ry, Rz per-axis gains (Rz negated)

# === Timer ===
TIMER_INTERVAL_MS = 4                        # polling interval (250 Hz)

# === HID data queue ===
DATA_QUEUE_SIZE = 256                        # max frames buffered between thread and UI

# === 3DxWare driver config path (uses APPDATA env var) ===
DRIVER_CFG_RELATIVE = r"3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml"

# === Axis index mapping (SpaceExplorer native order) ===
# Translation axis indices: Tx=0, Ty=1, Tz=2
# Rotation axis indices:    Rx=0, Ry=1, Rz=2
# Channel button order:     [Tx, Ty, Tz, Rx, Ry, Rz]

# Channel-mask swap: SpaceExplorer Ty/Tz and Ry/Rz are swapped relative to Houdini
# Button index -> raw axis index mapping for translation: [Tx→0, Ty→2, Tz→1]
CH_MASK_T_MAP = [0, 2, 1]
# Button index -> raw axis index mapping for rotation:    [Rx→0, Ry→2, Rz→1]
CH_MASK_R_MAP = [0, 2, 1]

# Houdini world coordinate remapping from raw values:
#   hx = T[0]        (Tx → Houdini X, same direction)
#   hy = -T[2]       (Tz → Houdini Y, negated: SpaceMouse +down → Houdini -Y)
#   hz = T[1]        (Ty → Houdini Z, same direction)
#   hrx = R[0]       (Rx → Houdini RX)
#   hry = -R[2]      (Rz → Houdini RY, negated)
#   hrz = R[1]       (Ry → Houdini RZ)
