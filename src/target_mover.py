"""
Movement application for SpaceMouse target types.
Applies translation/rotation deltas to the detected target:
  - OBJ node (worldTransform manipulation)
  - Parameter node (t/r parm tuples)
  - Channel parameters (scoped Rig Pose channels)
  - APEX control rig (graph_parms manipulation)
"""
import re
import numpy as np

import hou
import matrix_utils
import data_processor

# Regex patterns for t{n}x / r{n}x channel parameter names
_RE_TX = re.compile(r'(?:^|[_:])t\d*x$')
_RE_TY = re.compile(r'(?:^|[_:])t\d*y$')
_RE_TZ = re.compile(r'(?:^|[_:])t\d*z$')
_RE_RX = re.compile(r'(?:^|[_:])r\d*x$')
_RE_RY = re.compile(r'(?:^|[_:])r\d*y$')
_RE_RZ = re.compile(r'(?:^|[_:])r\d*z$')

_ch_printed = False
_apex_printed = False


def move_target(target, hx, hy, hz, hrx, hry, hrz, raw_t=None, cam_t=None,
                use_cam=True):
    """Dispatch movement to the appropriate handler.

    Args:
        target: (target_type, target_data) tuple from target_detector.
        hx, hy, hz: Houdini world translation deltas.
        hrx, hry, hrz: Houdini world rotation deltas.
        raw_t: Raw translation tuple (tx, ty, tz) for camera-relative calc.
        cam_t: Pre-computed camera-relative world translation tuple.
        use_cam: Whether to use camera-relative space (True) or
                 absolute world space (False).
    """
    kind = target[0]
    if kind == 'ch':
        _move_ch_parms(target[1], hx, hy, hz, hrx, hry, hrz, cam_t=cam_t)
    elif kind == 'parm':
        _move_parm_node(target[1], hx, hy, hz, hrx, hry, hrz, cam_t=cam_t)
    elif kind == 'apex':
        _move_apex_controls(target[1], hx, hy, hz, hrx, hry, hrz,
                            raw_t, use_cam)
    elif kind == 'obj':
        _move_obj_node(target[1], hx, hy, hz, hrx, hry, hrz, cam_t=cam_t)


# === Channel parameter movement ===

def _move_ch_parms(parms, tx, ty, tz, rx, ry, rz, cam_t=None):
    """Move scoped channel parameters (Rig Pose or general channels).

    Groups parameters by numeric index (bone number) so that multiple
    selected Rig Pose bones all move together. Each group gets its own
    camera-relative bone-world-rotation conversion.
    """
    global _ch_printed

    # Group parameters by bone index: {idx: {'t': [px,py,pz], 'r': [px,py,pz]}}
    groups = {}
    unindexed_t = [None, None, None]
    unindexed_r = [None, None, None]

    for p in parms:
        name = p.name().lower()
        m = re.search(r'(\d+)', name)
        idx = int(m.group(1)) if m else -1

        if idx >= 0:
            if idx not in groups:
                groups[idx] = {'t': [None, None, None], 'r': [None, None, None]}
            g = groups[idx]
        else:
            # Non-indexed parms: use unindexed slots
            if _RE_TX.search(name):
                unindexed_t[0] = p
            elif _RE_TY.search(name):
                unindexed_t[1] = p
            elif _RE_TZ.search(name):
                unindexed_t[2] = p
            if _RE_RX.search(name):
                unindexed_r[0] = p
            elif _RE_RY.search(name):
                unindexed_r[1] = p
            elif _RE_RZ.search(name):
                unindexed_r[2] = p
            continue

        if _RE_TX.search(name):
            g['t'][0] = p
        elif _RE_TY.search(name):
            g['t'][1] = p
        elif _RE_TZ.search(name):
            g['t'][2] = p
        if _RE_RX.search(name):
            g['r'][0] = p
        elif _RE_RY.search(name):
            g['r'][1] = p
        elif _RE_RZ.search(name):
            g['r'][2] = p

    # Include unindexed parms as group -1
    if any(unindexed_t) or any(unindexed_r):
        groups[-1] = {'t': unindexed_t, 'r': unindexed_r}

    # --- Apply movement to each bone group ---
    bone_names = []
    for idx in sorted(groups.keys()):
        g = groups[idx]
        t_map = g['t']
        r_map = g['r']

        # Camera-relative: convert through this bone's world orientation
        cur_tx, cur_ty, cur_tz = tx, ty, tz
        if cam_t is not None and idx >= 0 and all(t_map):
            world_rot = _get_rigpose_world_rot(t_map[0], idx)
            wd = np.array(cam_t)
            local_delta = wd @ world_rot.T
            cur_tx, cur_ty, cur_tz = local_delta[0], local_delta[1], local_delta[2]
        elif cam_t is not None:
            cur_tx, cur_ty, cur_tz = cam_t[0], cam_t[1], cam_t[2]

        if not _ch_printed and idx >= 0:
            bone_names.append("b{}".format(idx))

        if all(t_map):
            t_map[0].set(t_map[0].eval() + cur_tx)
            t_map[1].set(t_map[1].eval() + cur_ty)
            t_map[2].set(t_map[2].eval() + cur_tz)

        if all(r_map):
            rn_name = r_map[1].name() if r_map[1] else ''
            ry_val = -ry if re.search(r'\d', rn_name) else ry
            if abs(rx) > 1e-10:
                r_map[0].set(r_map[0].eval() + rx)
            if abs(ry) > 1e-10:
                r_map[1].set(r_map[1].eval() + ry_val)
            if abs(rz) > 1e-10:
                r_map[2].set(r_map[2].eval() + rz)

    if not _ch_printed and bone_names:
        rig_tag = '[rigpose {} bones]'.format(len(bone_names))
        print("[SpaceMouse] {} {}".format(rig_tag, ' '.join(bone_names)))
        _ch_printed = True
    elif not _ch_printed:
        # Non-indexed fallback: print first group's names
        g = groups.get(-1)
        if g:
            t_names = [g['t'][i].name() if g['t'][i] else '-' for i in range(3)]
            r_names = [g['r'][i].name() if g['r'][i] else '-' for i in range(3)]
            print("[SpaceMouse] T:{} {} {} | R:{} {} {}".format(
                t_names[0], t_names[1], t_names[2],
                r_names[0], r_names[1], r_names[2]))
            _ch_printed = True


def _get_rigpose_world_rot(parm, pt_idx):
    """Read world rotation of a Rig Pose bone point from geometry output.

    Args:
        parm: A channel parameter belonging to the Rig Pose node.
        pt_idx: Index of the bone point.

    Returns:
        3x3 numpy rotation matrix.
    """
    try:
        node = parm.node()
        geo = node.geometry() if hasattr(node, 'geometry') else None
        if geo is None and hasattr(node, 'displayNode'):
            dn = node.displayNode()
            geo = dn.geometry() if dn else None
        if geo is None:
            return np.eye(3)

        pts = list(geo.points())
        if pt_idx >= len(pts):
            return np.eye(3)

        pt = pts[pt_idx]
        try:
            xf = pt.attribValue('transform')
            if xf is not None and hasattr(xf, '__len__') and len(xf) == 9:
                rot33 = np.array(xf, dtype=np.float64).reshape(3, 3)
            else:
                rot33 = np.eye(3)
        except Exception:
            rot33 = np.eye(3)

        return rot33
    except Exception:
        return np.eye(3)


# === Parameter tuple movement ===

def _move_parm_node(node, tx, ty, tz, rx, ry, rz, cam_t=None):
    """Move a node via its 't' and 'r' parameter tuples."""
    if cam_t is not None:
        tx, ty, tz = cam_t[0], cam_t[1], cam_t[2]

    t_tuple = node.parmTuple('t')
    if t_tuple and len(t_tuple) >= 3:
        vals = t_tuple.eval()
        t_tuple.set((vals[0] + tx, vals[1] + ty, vals[2] + tz))

    r_tuple = node.parmTuple('r')
    if r_tuple and len(r_tuple) >= 3:
        vals = r_tuple.eval()
        nrx, nry, nrz = vals[0], vals[1], vals[2]
        if abs(rx) > 1e-10:
            nrx += rx
        if abs(ry) > 1e-10:
            nry += ry
        if abs(rz) > 1e-10:
            nrz += rz
        r_tuple.set((nrx, nry, nrz))


# === APEX control rig movement ===

def _move_apex_controls(target, tx, ty, tz, rx, ry, rz, raw_t=None,
                        use_cam=True):
    """Move APEX control rig.

    Two modes:
      - Cam Space (use_cam=True):  raw_t values are camera-relative.
        Convert to world delta via viewport camera matrix, then to
        each control's local space via skeleton world rotation.
      - Abs Space (use_cam=False): tx/ty/tz are already in Houdini
        world space. Apply directly as world delta — no camera matrix.

    Note on Channel List:
      APEX rig.graph_parms and the Channel List are separate parameter
      layers. Writing to graph_parms updates the internal graph, but the
      Channel List UI only refreshes when the APEX state handle pushes
      values (user interaction). Calling state.runSceneCallbacks()
      triggers scene re-evaluation but does NOT force a Channel List
      refresh. This is a known APEX limitation.
    """
    global _apex_printed

    state, ctrl_paths, skel_lookup = target
    if not ctrl_paths:
        return

    try:
        from apex.control_2 import controlRigPath
        ctrl_mgr = state.control_manager
        scene = state.scene

        # Compute world-space translation delta
        if use_cam:
            # Camera-relative: convert raw SpaceMouse values to world delta
            viewport = _get_viewport()
            cam_mat = matrix_utils.mat4_to_numpy(viewport.viewTransform()) if viewport else np.eye(4)
            cam_rot = cam_mat[:3, :3]
            if raw_t is not None:
                rtx, rty, rtz = raw_t
                world_delta = -rtx * cam_rot[0, :] + rtz * cam_rot[1, :] + rty * cam_rot[2, :]
            else:
                world_delta = tx * cam_rot[0, :] + ty * cam_rot[1, :] + tz * cam_rot[2, :]
        else:
            # Absolute world space: tx/ty/tz are Houdini world coords
            world_delta = np.array([tx, ty, tz], dtype=np.float64)

        for ctrl_path in ctrl_paths:
            rig_path = controlRigPath(ctrl_path)
            rig = scene.getData(rig_path)
            if rig is None:
                continue
            ctrl_map = ctrl_mgr.getControlMapping(ctrl_path)

            ctrl_name = ctrl_path.rsplit('/', 1)[-1] if '/' in ctrl_path else ctrl_path

            world_mat = skel_lookup.get(ctrl_name)
            world_rot = world_mat[:3, :3] if world_mat is not None else np.eye(3)

            # Convert world delta to control local space
            local_delta = world_delta @ world_rot.T

            # Current local values from graph_parms
            cur_t = hou.Vector3(0, 0, 0)
            cur_r = hou.Vector3(0, 0, 0)
            if ctrl_map.t:
                v = rig.graph_parms.get(ctrl_map.t)
                if v is not None:
                    if isinstance(v, hou.Vector3):
                        cur_t = v
                    else:
                        cur_t = hou.Vector3(v)
            if ctrl_map.r:
                v = rig.graph_parms.get(ctrl_map.r)
                if v is not None:
                    if isinstance(v, hou.Vector3):
                        cur_r = v
                    else:
                        cur_r = hou.Vector3(v)

            if not _apex_printed:
                has_skel = "skel" if world_mat is not None else "world"
                mode = "Cam" if use_cam else "Abs"
                print("[SpaceMouse] APEX [{}] ctrl={} [{}] T={} R={}".format(
                    mode, ctrl_name, has_skel, cur_t, cur_r))
                _apex_printed = True

            if ctrl_map.t and (abs(tx) > 1e-10 or abs(ty) > 1e-10 or abs(tz) > 1e-10):
                rig.graph_parms[ctrl_map.t] = hou.Vector3(
                    cur_t[0] + local_delta[0],
                    cur_t[1] + local_delta[1],
                    cur_t[2] + local_delta[2])

            if ctrl_map.r and (abs(rx) > 1e-10 or abs(ry) > 1e-10 or abs(rz) > 1e-10):
                rig.graph_parms[ctrl_map.r] = hou.Vector3(
                    cur_r[0] + rx, cur_r[1] + ry, cur_r[2] + rz)

        state.runSceneCallbacks()

    except Exception:
        import traceback
        traceback.print_exc()


# === OBJ node movement ===

def _move_obj_node(node, tx, ty, tz, rx, ry, rz, cam_t=None):
    """Move an OBJ node via its worldTransform.

    Applies translation along local axes and rotation via Rodrigues formula.
    """
    if cam_t is not None:
        tx, ty, tz = cam_t[0], cam_t[1], cam_t[2]

    obj_mat = matrix_utils.mat4_to_numpy(node.worldTransform())
    obj_pos = obj_mat[3, :3].copy()
    obj_rot = obj_mat[:3, :3].copy()

    # Translate along local axes
    obj_pos += tx * obj_rot[0, :] + ty * obj_rot[1, :] + tz * obj_rot[2, :]

    # Rotate around local axes via Rodrigues
    if abs(ry) > 1e-10:
        obj_rot = obj_rot @ matrix_utils.rodrigues(np.array([0., 1., 0.]), np.radians(ry)).T
    if abs(rx) > 1e-10:
        obj_rot = obj_rot @ matrix_utils.rodrigues(np.array([1., 0., 0.]), np.radians(rx)).T
    if abs(rz) > 1e-10:
        obj_rot = obj_rot @ matrix_utils.rodrigues(np.array([0., 0., 1.]), np.radians(rz)).T

    obj_rot = matrix_utils.orthonormalize(obj_rot)
    node.setWorldTransform(matrix_utils.numpy_to_mat4(obj_pos, obj_rot))


# === Viewport helper ===

def _get_viewport():
    """Get the current scene viewer's active viewport.

    Returns:
        hou.GeometryViewport or None.
    """
    try:
        desktop = hou.ui.curDesktop()
        viewer = desktop.paneTabOfType(hou.paneTabType.SceneViewer)
        return viewer.curViewport() if viewer else None
    except Exception:
        return None


# === Reset helpers ===

def reset_move_state():
    """Reset module-level state (call on stop)."""
    global _ch_printed, _apex_printed
    _ch_printed = False
    _apex_printed = False
