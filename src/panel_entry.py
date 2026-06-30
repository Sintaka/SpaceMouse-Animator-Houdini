"""SpaceMouse Houdini Panel - Entry Point

Paste this entire script into a Houdini Python Panel.
Edit SRC_PATH below to point to your src directory.
"""
import sys
import os

# ============================================================
# USER CONFIG: Set this to the absolute path of the src folder
# ============================================================
SRC_PATH = r"D:\code\dev\Houdini\spaceMouse1\src"

# ============================================================
# Path validation
# ============================================================
if not os.path.isdir(SRC_PATH):
    raise FileNotFoundError(
        "SpaceMouse src directory not found:\n"
        "  {}\n"
        "Please edit SRC_PATH in the panel script to point to"
        " the correct location.".format(SRC_PATH)
    )

if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

# ============================================================
# Import and expose the Houdini Python Panel interface
# ============================================================
from spacemouse_controller import create_interface as _create_interface


def createInterface():
    return _create_interface()
