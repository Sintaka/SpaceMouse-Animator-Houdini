"""
Matrix utility functions for SpaceMouse Houdini operations.
Converts between hou.Matrix4/Matrix3 and numpy arrays.
Provides Rodrigues rotation and Gram-Schmidt orthonormalization.
"""
import numpy as np

try:
    import hou
    _HOU_AVAILABLE = True
except ImportError:
    _HOU_AVAILABLE = False


def mat4_to_numpy(m):
    """Convert hou.Matrix4 to 4x4 numpy array (row-major)."""
    return np.array([
        [m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3)],
        [m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3)],
        [m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3)],
        [m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3)],
    ], dtype=np.float64)


def numpy_to_mat4(pos, rot):
    """Build hou.Matrix4 from position (3,) and rotation (3x3) numpy arrays."""
    m = hou.Matrix4()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
        m.setAt(i, 3, 0.0)
    m.setAt(3, 0, float(pos[0]))
    m.setAt(3, 1, float(pos[1]))
    m.setAt(3, 2, float(pos[2]))
    m.setAt(3, 3, 1.0)
    return m


def numpy_to_mat3(rot):
    """Build hou.Matrix3 from 3x3 numpy array."""
    m = hou.Matrix3()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
    return m


def rodrigues(axis, angle_rad):
    """Rodrigues rotation formula: build 3x3 rotation matrix from axis + angle.

    Args:
        axis: Unit or non-unit rotation axis (3,).
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation matrix (numpy float64).
    """
    axis = axis / np.linalg.norm(axis)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    t = 1.0 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,     t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z,   t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y,   t*y*z + s*x, t*z*z + c],
    ], dtype=np.float64)


def orthonormalize(rot):
    """Gram-Schmidt orthonormalization of a 3x3 rotation matrix.

    Ensures the matrix is a proper rotation (orthonormal columns).
    Forward (row 2) is preserved; right (row 0) is projected off forward;
    up (row 1) is computed from cross product.
    """
    fwd = rot[2, :].copy()
    fwd /= np.linalg.norm(fwd)
    right = rot[0, :].copy()
    right -= np.dot(right, fwd) * fwd
    right /= np.linalg.norm(right)
    up = np.cross(fwd, right)
    up /= np.linalg.norm(up)
    out = rot.copy()
    out[0, :] = right
    out[1, :] = up
    out[2, :] = fwd
    return out


def tuple_to_mat4(t, col_major=False):
    """Convert 16-element tuple to 4x4 numpy matrix.

    Args:
        t: 16-element sequence (row-major by default, Houdini format).
        col_major: If True, transpose the result (column-major input).

    Returns:
        4x4 numpy array.
    """
    m = np.array(t, dtype=np.float64).reshape(4, 4)
    if col_major:
        m = m.T
    return m
