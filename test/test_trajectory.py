import numpy as np
import pytest

from dvrk_simulator_base.trajectory import JointTrajectory


def test_joints_finish_together_with_velocity_limits():
    trajectory = JointTrajectory(
        start=[0.0, 0.0],
        goal=[1.0, 2.0],
        velocity_limits=[1.0, 1.0],
        start_time=10.0,
    )
    assert trajectory.duration == pytest.approx(2.0)
    middle = trajectory.sample(11.0)
    np.testing.assert_allclose(middle.position, [0.5, 1.0])
    np.testing.assert_allclose(middle.velocity, [0.5, 1.0])
    assert not middle.complete
    end = trajectory.sample(12.0)
    np.testing.assert_allclose(end.position, [1.0, 2.0])
    np.testing.assert_allclose(end.velocity, [0.0, 0.0])
    assert end.complete


def test_zero_distance_trajectory_completes_immediately():
    trajectory = JointTrajectory([1.0], [1.0], [1.0], start_time=0.0)
    assert trajectory.sample(0.0).complete


def test_invalid_velocity_limit_is_rejected():
    with pytest.raises(ValueError, match="positive"):
        JointTrajectory([0.0], [1.0], [0.0], start_time=0.0)
