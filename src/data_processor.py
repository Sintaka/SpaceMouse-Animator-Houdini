"""
Coordinate processing pipeline for SpaceMouse raw HID data.
Converts raw int16 axis values into Houdini world-space coordinates
by applying sensitivity, per-axis gain, channel masking, and axis remapping.

Pure math module — no Qt or Houdini dependency.
"""
import numpy as np
import config


def process_frame(raw_data, t_sensitivity, r_sensitivity, t_gains, r_gains,
                  channel_mask):
    """Convert a raw HID data dict into Houdini world-space deltas.

    Args:
        raw_data: dict with keys 'translation' [tx,ty,tz], 'rotation' [rx,ry,rz].
        t_sensitivity: float, translation sensitivity multiplier.
        r_sensitivity: float, rotation sensitivity multiplier.
        t_gains: list of 3 floats, per-axis translation gains [Tx, Ty, Tz].
        r_gains: list of 3 floats, per-axis rotation gains [Rx, Ry, Rz].
        channel_mask: list of 6 bools, [Tx, Ty, Tz, Rx, Ry, Rz] on/off.

    Returns:
        tuple (hx, hy, hz, hrx, hry, hrz):
            hx, hy, hz  — Houdini world translation deltas.
            hrx, hry, hrz — Houdini world rotation deltas.
    """
    tx, ty, tz = raw_data['translation']
    rx, ry, rz = raw_data['rotation']

    # Normalise and apply sensitivity + per-axis gain
    gt = np.array(t_gains, dtype=np.float64)
    gr = np.array(r_gains, dtype=np.float64)
    tv = np.array([tx, ty, tz], dtype=np.float64) / config.AXIS_RANGE * t_sensitivity * gt
    rv = np.array([rx, ry, rz], dtype=np.float64) / config.AXIS_RANGE * r_sensitivity * gr

    # Apply channel mask with SpaceExplorer axis swap:
    #   UI button order:  [Tx, Ty, Tz, Rx, Ry, Rz]
    #   Raw axis mapping: tv[0]=Tx, tv[2]=Ty (swapped), tv[1]=Tz (swapped)
    #                      rv[0]=Rx, rv[2]=Ry (swapped), rv[1]=Rz (swapped)
    cm = [1.0 if a else 0.0 for a in channel_mask]
    t_mask = np.array([cm[i] for i in config.CH_MASK_T_MAP], dtype=np.float64)
    r_mask = np.array([cm[i + 3] for i in config.CH_MASK_R_MAP], dtype=np.float64)
    tv = tv * t_mask
    rv = rv * r_mask

    # Remap to Houdini world coordinates:
    #   hx = T[0]       (Tx → Houdini X)
    #   hy = -T[2]      (Tz → Houdini Y, negated)
    #   hz = T[1]       (Ty → Houdini Z)
    #   hrx = R[0]      (Rx → Houdini RX)
    #   hry = -R[2]     (Rz → Houdini RY, negated)
    #   hrz = R[1]      (Ry → Houdini RZ)
    hx, hy, hz = tv[0], -tv[2], tv[1]
    hrx, hry, hrz = rv[0], -rv[2], rv[1]

    return (hx, hy, hz, hrx, hry, hrz)


def compute_camera_delta(tv, viewport_mat):
    """Compute camera-relative world-space translation delta.

    Args:
        tv: Raw translation numpy array [tx, ty, tz] (after sensitivity/gain).
        viewport_mat: 4x4 numpy array from viewport.viewTransform().

    Returns:
        tuple (wx, wy, wz): World-space translation delta.
    """
    cr = viewport_mat[0, :3]   # camera right
    cu = viewport_mat[1, :3]   # camera up
    cf = viewport_mat[2, :3]   # camera forward

    rtx, rty, rtz = tv[0], tv[1], tv[2]
    cam_world = rtx * cr - rtz * cu + rty * cf
    return (cam_world[0], cam_world[1], cam_world[2])
