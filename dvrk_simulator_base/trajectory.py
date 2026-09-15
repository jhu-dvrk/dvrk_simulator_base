"""Synchronized velocity-limited joint-space trajectories."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    position: np.ndarray
    velocity: np.ndarray
    complete: bool


class JointTrajectory:
    """Linear trajectory in which all joints finish simultaneously."""

    def __init__(self, start, goal, velocity_limits, start_time: float) -> None:
        self.start = np.asarray(start, dtype=float).copy()
        self.goal = np.asarray(goal, dtype=float).copy()
        self.velocity_limits = np.asarray(velocity_limits, dtype=float).copy()
        if self.start.ndim != 1 or self.goal.shape != self.start.shape:
            raise ValueError("trajectory start and goal must have matching vector shapes")
        if self.velocity_limits.shape != self.start.shape:
            raise ValueError("velocity limits must match the joint vector")
        if not all(
            np.all(np.isfinite(value))
            for value in (self.start, self.goal, self.velocity_limits)
        ):
            raise ValueError("trajectory values must be finite")
        if np.any(self.velocity_limits <= 0.0):
            raise ValueError("trajectory velocity limits must be positive")
        if not np.isfinite(start_time):
            raise ValueError("trajectory start time must be finite")
        self.start_time = float(start_time)
        self.delta = self.goal - self.start
        self.duration = float(np.max(np.abs(self.delta) / self.velocity_limits))
        self.start.setflags(write=False)
        self.goal.setflags(write=False)
        self.velocity_limits.setflags(write=False)
        self.delta.setflags(write=False)

    def sample(self, now: float) -> TrajectorySample:
        elapsed = max(0.0, float(now) - self.start_time)
        if self.duration == 0.0 or elapsed >= self.duration:
            position = self.goal.copy()
            velocity = np.zeros_like(self.goal)
            complete = True
        else:
            alpha = elapsed / self.duration
            position = self.start + alpha * self.delta
            velocity = self.delta / self.duration
            complete = False
        position.setflags(write=False)
        velocity.setflags(write=False)
        return TrajectorySample(position, velocity, complete)
