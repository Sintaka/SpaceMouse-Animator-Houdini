"""
Debug logging utilities.
Redirects print() output to both stdout and a debug log file.
The log file is cleared at the start of each session.
Path is resolved relative to src/ via config.get_src_dir().
"""
import sys
import os
import io
from datetime import datetime

import config

_ORIG_PRINT = print


def setup_debug_logging(log_dir=None):
    """Install tee-print to duplicate output to debug log file.

    Clears any existing log and writes a fresh session header.
    All subsequent print() calls are mirrored to the log file.

    Args:
        log_dir: Optional custom log directory. If None, defaults to
                 <src>/debug/ resolved via config.get_src_dir().

    Returns:
        str: Absolute path to the debug log file.
    """
    if log_dir is None:
        log_dir = os.path.join(config.get_src_dir(), "debug")

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_path = os.path.join(log_dir, "spacemouse_debug.log")

    # Clear log and write fresh session header
    with open(log_path, 'w', encoding='utf-8') as f:
        f.write("=== SpaceMouse Debug Log ===\n")
        f.write("Session: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        f.write("Src dir: {}\n\n".format(config.get_src_dir()))

    # Patch builtins.print to tee output to the log file
    import builtins

    def _tee_print(*args, **kwargs):
        _ORIG_PRINT(*args, **kwargs)
        try:
            buf = io.StringIO()
            _ORIG_PRINT(*args, file=buf, **kwargs)
            with open(log_path, 'a', encoding='utf-8') as f:
                f.write(buf.getvalue())
        except Exception:
            pass

    builtins.print = _tee_print

    return log_path


__all__ = ["setup_debug_logging"]

