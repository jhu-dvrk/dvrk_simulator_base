"""Validated, immutable value types shared by simulator backends."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _readonly_vector(value: object, size: int, field: str) -> np.ndarray:
    array = np.array(value, dtype=float, copy=True)
    if array.shape != (size,):
        raise ValueError(f"{field} must have shape ({size},), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field} must contain only finite values")
    array.setflags(write=False)
    return array


def _readonly_matrix(value: object, shape: tuple[int, int], field: str) -> np.ndarray:
    array = np.array(value, dtype=float, copy=True)
    if array.shape != shape:
        raise ValueError(f"{field} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field} must contain only finite values")
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class Pose:
    """Rigid pose with position in meters and a 3-by-3 rotation matrix."""

    position: np.ndarray
    orientation: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _readonly_vector(self.position, 3, "position"))
        orientation = _readonly_matrix(self.orientation, (3, 3), "orientation")
        if not np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-7):
            raise ValueError("orientation must be orthonormal")
        if not np.isclose(np.linalg.det(orientation), 1.0, atol=1e-7):
            raise ValueError("orientation must be a proper rotation matrix")
        object.__setattr__(self, "orientation", orientation)


@dataclass(frozen=True)
class Twist:
    """Linear velocity in m/s and angular velocity in rad/s."""

    linear: np.ndarray
    angular: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear", _readonly_vector(self.linear, 3, "linear"))
        object.__setattr__(self, "angular", _readonly_vector(self.angular, 3, "angular"))


@dataclass(frozen=True)
class JointState:
    """A joint vector whose arrays follow ``names`` order."""

    names: tuple[str, ...]
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray | None = None

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.names)
        if not names:
            raise ValueError("names cannot be empty")
        if any(not name for name in names):
            raise ValueError("joint names cannot be empty")
        if len(set(names)) != len(names):
            raise ValueError("joint names must be unique")
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "position", _readonly_vector(self.position, len(names), "position"))
        object.__setattr__(self, "velocity", _readonly_vector(self.velocity, len(names), "velocity"))
        if self.effort is not None:
            object.__setattr__(self, "effort", _readonly_vector(self.effort, len(names), "effort"))


@dataclass(frozen=True)
class IKResult:
    """Normalized result returned by a backend inverse-kinematics solver."""

    position: np.ndarray
    success: bool
    iterations: int | None = None
    position_error: float | None = None
    orientation_error: float | None = None
    message: str = ""

    def __post_init__(self) -> None:
        position = np.array(self.position, dtype=float, copy=True)
        if position.ndim != 1:
            raise ValueError("position must be a one-dimensional joint vector")
        if not np.all(np.isfinite(position)):
            raise ValueError("position must contain only finite values")
        position.setflags(write=False)
        object.__setattr__(self, "position", position)
        if self.iterations is not None and self.iterations < 0:
            raise ValueError("iterations cannot be negative")
        for field in ("position_error", "orientation_error"):
            value = getattr(self, field)
            if value is not None and (not np.isfinite(value) or value < 0.0):
                raise ValueError(f"{field} must be finite and non-negative")
