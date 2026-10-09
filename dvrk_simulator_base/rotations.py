"""Rotation conversions with explicit ROS XYZW quaternion ordering."""

from __future__ import annotations

import math

import numpy as np


def quaternion_matrix_xyzw(quaternion: object) -> np.ndarray:
    """Convert an ``(x, y, z, w)`` quaternion to a rotation matrix."""
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,):
        raise ValueError(f"quaternion must have shape (4,), got {value.shape}")
    if not np.all(np.isfinite(value)):
        raise ValueError("quaternion must contain only finite values")
    norm = float(np.linalg.norm(value))
    if norm == 0.0:
        raise ValueError("quaternion cannot be zero")
    x, y, z, w = value / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_to_quaternion_xyzw(rotation: object) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to a normalized XYZW quaternion."""
    value = np.asarray(rotation, dtype=float)
    if value.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {value.shape}")
    if not np.all(np.isfinite(value)):
        raise ValueError("rotation must contain only finite values")
    if not np.allclose(value.T @ value, np.eye(3), atol=1e-7):
        raise ValueError("rotation must be orthonormal")
    if not np.isclose(np.linalg.det(value), 1.0, atol=1e-7):
        raise ValueError("rotation must be a proper rotation matrix")

    trace = float(np.trace(value))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (value[2, 1] - value[1, 2]) / scale
        y = (value[0, 2] - value[2, 0]) / scale
        z = (value[1, 0] - value[0, 1]) / scale
    elif value[0, 0] > value[1, 1] and value[0, 0] > value[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2])
        w = (value[2, 1] - value[1, 2]) / scale
        x = 0.25 * scale
        y = (value[0, 1] + value[1, 0]) / scale
        z = (value[0, 2] + value[2, 0]) / scale
    elif value[1, 1] > value[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2])
        w = (value[0, 2] - value[2, 0]) / scale
        x = (value[0, 1] + value[1, 0]) / scale
        y = 0.25 * scale
        z = (value[1, 2] + value[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1])
        w = (value[1, 0] - value[0, 1]) / scale
        x = (value[0, 2] + value[2, 0]) / scale
        y = (value[1, 2] + value[2, 1]) / scale
        z = 0.25 * scale
    result = np.asarray([x, y, z, w], dtype=float)
    result /= np.linalg.norm(result)
    return tuple(float(component) for component in result)


def rotation_to_quaternion_wxyz(rotation: object) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to scalar-first ordering for Isaac Sim."""
    x, y, z, w = rotation_to_quaternion_xyzw(rotation)
    return (w, x, y, z)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if axis.shape != (3,) or not np.isfinite(axis).all() or norm == 0.0:
        raise ValueError("rotation axis must be a finite nonzero three-vector")
    axis = axis / norm
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return np.array(
        [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ]
    )
