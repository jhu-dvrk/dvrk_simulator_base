import numpy as np
import pytest

from dvrk_simulator_base.types import IKResult, JointState, Pose, Twist


def test_pose_owns_read_only_arrays():
    source = np.array([1.0, 2.0, 3.0])
    pose = Pose(source, np.eye(3))
    source[0] = 10.0

    np.testing.assert_allclose(pose.position, [1.0, 2.0, 3.0])
    assert not pose.position.flags.writeable
    assert not pose.orientation.flags.writeable


@pytest.mark.parametrize(
    "position, orientation",
    [([0.0, 0.0], np.eye(3)), ([0.0, 0.0, 0.0], np.zeros((3, 3)))],
)
def test_pose_rejects_invalid_shapes_and_rotations(position, orientation):
    with pytest.raises(ValueError):
        Pose(position, orientation)


def test_twist_and_joint_state_validate_vector_lengths():
    twist = Twist(np.zeros(3), np.ones(3))
    assert not twist.angular.flags.writeable

    with pytest.raises(ValueError, match="shape"):
        JointState(("yaw", "pitch"), np.zeros(1), np.zeros(2))
    with pytest.raises(ValueError, match="unique"):
        JointState(("yaw", "yaw"), np.zeros(2), np.zeros(2))


def test_ik_result_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        IKResult(np.array([np.nan]), False)
