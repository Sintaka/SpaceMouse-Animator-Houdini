"""
3DxWare driver XML config toggle.
Reads/writes SideFX_HoudiniFX.xml to enable or disable the 3DxWare driver
so it does not fight with the custom SpaceMouse receiver for HID access.
"""
import os
import xml.etree.ElementTree as ET


def _get_driver_cfg_path():
    """Resolve the 3DxWare config file path using APPDATA env var."""
    appdata = os.environ.get('APPDATA', '')
    return os.path.join(appdata,
                        r'3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml')


def is_driver_enabled():
    """Check if the 3DxWare driver is currently enabled for Houdini.

    Returns:
        bool: True if at least one Axis is enabled, False otherwise.
              Returns True (assumed enabled) if config file is missing.
    """
    cfg_path = _get_driver_cfg_path()
    if not os.path.exists(cfg_path):
        return True  # assume enabled if config missing

    try:
        tree = ET.parse(cfg_path)
        for axis in tree.getroot().iter('Axis'):
            en = axis.find('Enabled')
            if en is not None and en.text and en.text.lower() == 'true':
                return True
        return False
    except Exception:
        return True


def set_driver_enabled(enable):
    """Enable or disable the 3DxWare driver for Houdini.

    Finds all <Axis> elements in the config and sets their <Enabled> tag.

    Args:
        enable: bool — True to enable driver, False to disable.

    Returns:
        bool: True on success, False if config missing or on error.
    """
    cfg_path = _get_driver_cfg_path()
    if not os.path.exists(cfg_path):
        return False

    try:
        tree = ET.parse(cfg_path)
        changed = 0
        for axis in tree.getroot().iter('Axis'):
            en = axis.find('Enabled')
            if en is not None:
                new_val = 'true' if enable else 'false'
                if en.text != new_val:
                    en.text = new_val
                    changed += 1
        if changed > 0:
            tree.write(cfg_path, encoding='UTF-8', xml_declaration=True)
        return True
    except Exception:
        return False
