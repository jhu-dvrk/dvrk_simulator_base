"""Immutable state snapshots passed from simulator owners to ROS threads."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import JointState, Pose, Twist


@dataclass(frozen=True)
class OperatingStateSnapshot:
    state: str
    is_homed: bool
    is_busy: bool


@dataclass(frozen=True)
class ArmSnapshot:
    sequence: int
    simulation_time: float
    valid: bool
    measured_js: JointState
    setpoint_js: JointState
    measured_cp_world: Pose
    setpoint_cp_world: Pose
    measured_cv_world: Twist
    jaw_measured: float | None
    jaw_setpoint: float | None
    operating_state: OperatingStateSnapshot
    operating_state_event: bool = False

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("snapshot sequence cannot be negative")
        if not np.isfinite(self.simulation_time) or self.simulation_time < 0.0:
            raise ValueError("simulation time must be finite and non-negative")
        for name in ("jaw_measured", "jaw_setpoint"):
            value = getattr(self, name)
            if value is not None and not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
