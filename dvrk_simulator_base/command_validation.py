"""Validation and normalization for CRTK joint command messages."""

from __future__ import annotations

from typing import Iterable

import numpy as np
import PyKDL


def joint_positions_from_message(message, expected_names: Iterable[str]) -> np.ndarray:
    """Return joint positions ordered according to the configured model."""
    expected = tuple(expected_names)
    if not message.position:
        raise ValueError("joint command has no position values")
    values = np.asarray(message.position, dtype=float)
    if message.name:
        names = tuple(message.name)
        positional_names = tuple(str(index) for index in range(len(expected)))
        if names == positional_names:
            # cisst's generic JointState bridge uses numeric names for an
            # ordered joint vector rather than the model-specific names.
            pass
        elif len(names) != len(set(names)) or set(names) != set(expected):
            raise ValueError(f"joint command names {names} do not match {expected}")
        else:
            values = np.asarray(
                [message.position[names.index(name)] for name in expected],
                dtype=float,
            )
    if values.shape != (len(expected),):
        raise ValueError("joint command has the wrong number of positions")
    if not np.all(np.isfinite(values)):
        raise ValueError("joint command positions must be finite")
    return values


def jaw_position_from_message(message) -> float:
    """Return the single logical jaw position from a jaw command."""
    if not message.position:
        raise ValueError("jaw command has no position values")
    if len(message.position) != 1:
        raise ValueError("jaw command must contain exactly one position")
    value = float(message.position[0])
    if not np.isfinite(value):
        raise ValueError("jaw command position must be finite")
    return value


def pose_from_message(message) -> PyKDL.Frame:
    """Return a validated PyKDL.Frame from a ROS ``Pose`` or ``PoseStamped`` value."""
    import math

    value = message.pose if hasattr(message, "pose") else message
    if not (np.isfinite(value.position.x) and np.isfinite(value.position.y) and np.isfinite(value.position.z)):
        raise ValueError("Cartesian command position must be finite")
    qx = float(value.orientation.x)
    qy = float(value.orientation.y)
    qz = float(value.orientation.z)
    qw = float(value.orientation.w)
    if not (np.isfinite(qx) and np.isfinite(qy) and np.isfinite(qz) and np.isfinite(qw)):
        raise ValueError("Cartesian command orientation must be finite")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("quaternion cannot be zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    position = PyKDL.Vector(value.position.x, value.position.y, value.position.z)
    rotation = PyKDL.Rotation.Quaternion(qx, qy, qz, qw)
    return PyKDL.Frame(rotation, position)
