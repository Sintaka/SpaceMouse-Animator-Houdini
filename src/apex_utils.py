"""
APEX utility functions for SpaceMouse control rig movement.
Provides skeleton world-transform lookup from evaluated graph output,
and detection of scoped APEX control parameters.
"""
import numpy as np

import hou
import matrix_utils

_apex_dump_done = False


def get_apex_state():
    """Retrieve the current APEX state from the active Scene Viewer.

    Returns:
        APEX state object, or None if not available.
    """
    try:
        import apex
        sv = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
        if sv is None:
            return None
        kwargs = {}
        sv.runStateCommand('getState', kwargs)
        return kwargs.get('state')
    except Exception:
        return None


def get_scoped_controls_info(state):
    """Get human-readable info about the currently selected APEX controls.

    Args:
        state: APEX state object from get_apex_state().

    Returns:
        str: Multi-line info string for display.
    """
    info = ""
    try:
        ctrls = getattr(state, 'control_selection', None)
        if ctrls:
            info = "Controls: {}\n".format(ctrls)
            from apex.control_2 import controlRigPath
            cm = state.control_manager
            scene = state.scene
            for cp in ctrls:
                rp = controlRigPath(cp)
                rig = scene.getData(rp)
                if rig and cm:
                    cpm = cm.getControlMapping(cp)
                    if cpm.t:
                        info += "  {}: {}\n".format(cpm.t, rig.graph_parms.get(cpm.t))
                    if cpm.r:
                        info += "  {}: {}\n".format(cpm.r, rig.graph_parms.get(cpm.r))
        else:
            info = "No APEX controls selected"
    except Exception as e:
        info = "Error: {}".format(e)
    return info


def build_skel_lookup(state, ctrl_paths):
    """Build {control_name: world_matrix} from evaluated APEX graph output.

    Iterates over output ports of the APEX rig graph. For geometry outputs,
    reads point attributes 'name' and 'xform'/'transform'. For dict outputs,
    reads hou.Matrix4 values directly.

    Args:
        state: APEX state object.
        ctrl_paths: List of APEX control paths.

    Returns:
        dict mapping control name (str) → 4x4 numpy world matrix.
    """
    global _apex_dump_done
    lookup = {}
    try:
        from apex.control_2 import controlRigPath
        scene = state.scene
        if not ctrl_paths:
            return lookup

        rig_path = controlRigPath(ctrl_paths[0])
        rig = scene.getData(rig_path)
        if rig is None:
            return lookup

        g = rig.graph
        out_ports = g.outputPorts()

        for op in out_ports:
            try:
                pn = op.portName()
                data = g.getOutputData(pn)
            except Exception:
                continue
            if data is None:
                continue

            # Geometry output (from APEX Python Script or SOPs)
            if isinstance(data, hou.Geometry):
                npts = data.intrinsicValue('pointcount')
                if npts == 0:
                    continue
                pa = [a.name() for a in data.pointAttribs()]
                if not _apex_dump_done:
                    print("[SpaceMouse] {}: {}pts attrs={}".format(pn, npts, pa))

                has_name = 'name' in pa
                has_xf = any(a in pa for a in ('xform', 'transform'))
                if has_name and has_xf:
                    for pt in data.points():
                        try:
                            nm = pt.attribValue('name')
                            if not nm:
                                continue
                            for an in ('xform', 'transform'):
                                try:
                                    xf = pt.attribValue(an)
                                    if xf is not None and hasattr(xf, '__len__') and len(xf) in (12, 16):
                                        lookup[nm] = matrix_utils.tuple_to_mat4(xf)
                                        break
                                except Exception:
                                    pass
                        except Exception:
                            pass

            # Dict output (from APEX Script dict::Build)
            elif isinstance(data, dict) or hasattr(data, 'keys'):
                if not _apex_dump_done:
                    keys_sample = list(data.keys())[:10]
                    print("[SpaceMouse] {}: dict keys={}".format(pn, keys_sample))
                for k, v in data.items():
                    try:
                        if isinstance(v, hou.Matrix4):
                            lookup[k] = matrix_utils.mat4_to_numpy(v)
                    except Exception:
                        pass

        _apex_dump_done = True
    except Exception:
        pass

    return lookup
