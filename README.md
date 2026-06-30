# SpaceMouse for Houdini

[中文](README.zh-CN.md)

3Dconnexion SpaceExplorer driver for Houdini — camera navigation + object manipulation.

<img width="3550" height="1787" alt="SpaceMouse Houdini Panel" src="https://github.com/user-attachments/assets/9b56e377-3837-48cc-8dde-c7aae0a4bc4b" />

---

## Install

Install `hidapi` into Houdini's bundled Python:

```bash
"C:\Program Files\Side Effects Software\Houdini 21.0.729\python311\python.exe" -m pip install hidapi
```

## Setup

1. Open Houdini → **Windows** → **Python Panel Editor**
2. Paste the content of [`src/panel_entry.py`](src/panel_entry.py) into a new panel
3. **Edit `SRC_PATH`** near the top to point at your `src/` directory:

   ```python
   SRC_PATH = r"D:\code\dev\Houdini\spaceMouse1\src"
   ```

4. Click **Start** — the panel reads SpaceMouse HID directly, no external process needed.

> Previously the project used separate sender + receiver processes over UDP. The current version merges everything into modular Python files under `src/`.

## Files

```
src/
  panel_entry.py                # Paste this into Houdini Python Panel
  spacemouse_controller.py      # Orchestrator — wires HID + UI + processing
  ui_panel.py                   # Qt panel UI (no business logic)
  hid_reader.py                 # HID device reader (daemon thread → queue)
  data_processor.py             # Raw axis → delta values (sensitivity, gain, mask, remap)
  target_detector.py            # Auto-detect movable target (OBJ / parm / Rig Pose / APEX)
  target_mover.py               # Apply movement per target type
  driver_control.py             # Toggle 3DxWare driver via XML config
  config.py                     # Hardware IDs, sensitivity defaults, axis mapping
  matrix_utils.py               # numpy ↔ hou.Matrix conversions, Rodrigues
  apex_utils.py                 # APEX skeleton lookup, scoped-control detection
  debug_utils.py                # Tee-print to debug log
  STRUCTURE.txt                 # One-line module index
  debug/                        # Standalone test scripts
```

## UI

```
[Start] [Detect Parms]

3DxWare: [Driver: ON (Camera)]  [Cam Space / Abs Space]

   Tx Ty Tz Rx Ry Rz   T R   All None

Sensitivity (raw/350 * gain)
  T: [0.05]  R: [1.0]
  Tx  Ty  Tz   Rx  Ry  Rz
  [1] [1] [1]  [1] [1] [-1]
```

| Control | Function |
|---------|----------|
| **Start / Stop** | Begin/end HID reading |
| **Detect Parms** | Print current target parameters to console |
| **Driver ON / OFF** | Toggle 3DxWare axis enable — ON = camera, OFF = object |
| **Cam Space / Abs Space** | Camera-relative or world-absolute movement |
| **Tx Ty Tz Rx Ry Rz** | Per-axis toggle. Alt+Click = solo |
| **T / R** | Translation / Rotation master toggle |
| **All / None** | Enable / disable all 6 axes |

Per-axis gain: 6 spinboxes (-1.0 ~ 1.0). Negative = invert direction.

## Axis mapping (SpaceExplorer)

| Raw | Direction | Houdini |
|-----|-----------|---------|
| Tx | left(-) / right(+) | X |
| Ty | fwd(-) / back(+) | Z |
| Tz | down(+) / up(-) | Y |
| Rx | Pitch fwd(-) / back(+) | RX |
| Ry | Roll cw(-) / ccw(+) | RZ |
| Rz | Yaw cw(-) / ccw(+) | RY |

## Supported targets

| Type | Method |
|------|--------|
| OBJ node | `setWorldTransform()` |
| Transform SOP / Rig Pose | scoped parm → regex match → `parm.set()` |
| APEX Scene Animate | `rig.graph_parms` (local only) |
| LOP Transform | `parmTuple('t')` / `parmTuple('r')` |

## Driver mute

The panel toggles `<Enabled>` in `%APPDATA%\3Dconnexion\3DxWare\Cfg\SideFX_HoudiniFX.xml`. 3DxWare picks up changes live — no restart needed.

## Known issues

- **APEX**: `rig.graph_parms` writes are not undoable and Channel List does not auto-refresh. This is a Houdini API limitation (Python has no access to `PI_Handle`).
- **Port conflict** (legacy UDP only): if switching from an old `.pypanel`, restart Houdini twice to release the socket.
