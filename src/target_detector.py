"""
Target detection for SpaceMouse movement.
Determines what to move based on current Houdini selection/state.
Priority: scoped channel parms > parm tuple > APEX controls > OBJ node.
"""
import hou
import apex_utils

_last_target_kind = None


def detect_target():
    """Detect the current movable target in Houdini.

    Checks in priority order:
      1. Selected node with scoped channel parameters ('ch')
      2. Selected node with 't' parm tuple ('parm')
      3. APEX controls selected in scene viewer ('apex')
      4. Selected OBJ node with worldTransform ('obj')

    Returns:
        tuple (target_type, target_data) or None:
            ('obj',  node)
            ('parm', node)
            ('ch',   scoped_parms_list)
            ('apex', (state, ctrl_paths_list, skel_lookup_dict))
    """
    global _last_target_kind

    # 1. Check selected nodes
    try:
        sel = hou.selectedNodes()
        if sel:
            node = sel[0]

            # OBJ node with world transform
            if hasattr(node, 'worldTransform') and callable(getattr(node, 'worldTransform', None)):
                return ('obj', node)

            # Scoped channel parameters (Rig Pose, etc.)
            scoped = [p for p in node.parms() if p.isScoped()]
            if scoped:
                return ('ch', scoped)

            # Direct parameter tuple
            t_tuple = node.parmTuple('t')
            if t_tuple is not None and len(t_tuple) >= 3:
                return ('parm', node)
    except Exception:
        pass

    # 2. Check APEX controls
    try:
        state = apex_utils.get_apex_state()
        if state is not None:
            ctrls = getattr(state, 'control_selection', None)
            if ctrls:
                ctrl_list = list(ctrls)
                skel_lookup = apex_utils.build_skel_lookup(state, ctrl_list)
                return ('apex', (state, ctrl_list, skel_lookup))
    except Exception:
        pass

    _last_target_kind = None
    return None


def log_target(kind, detail):
    """Print target change if kind differs from last logged.

    Args:
        kind: Target type string ('obj', 'parm', 'ch', 'apex').
        detail: Human-readable detail string.
    """
    global _last_target_kind
    if _last_target_kind != kind:
        print("[SpaceMouse] target: [{}] {}".format(kind, detail))
        _last_target_kind = kind


def reset_target_log():
    """Reset the last-target-kind tracker (e.g. on stop)."""
    global _last_target_kind
    _last_target_kind = None
