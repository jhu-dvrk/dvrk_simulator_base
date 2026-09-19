"""Simulator-independent runtime kinematics contract."""

from __future__ import annotations

from typing import Protocol

import numpy as np
import PyKDL

from dvrk.types import IKResult, JointState


class KinematicsBackend(Protocol):
    """Operations every simulator arm exposes to the common command layer."""

    @property
    def joint_names(self) -> tuple[str, ...]: ...

    def measured_js(self) -> JointState: ...

    def setpoint_js(self) -> JointState: ...

    def measured_cp(self) -> PyKDL.Frame: ...

    def setpoint_cp(self) -> PyKDL.Frame: ...

    def measured_cv(self) -> PyKDL.Twist: ...

    def solve_ik(self, target: PyKDL.Frame, seed: np.ndarray | None = None) -> IKResult: ...

    def apply_joint_setpoint(self, position: np.ndarray) -> None: ...

    def step(self, dt: float) -> None: ...

    def reset(self, home: np.ndarray) -> None: ...
