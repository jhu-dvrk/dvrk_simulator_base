import numpy as np
import pytest
import PyKDL

from dvrk.types import Frame, IKResult, JointState, Pose, Twist


def test_pykdl_frame_and_twist_types():
    frame = Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(1.0, 2.0, 3.0))
    assert isinstance(frame, PyKDL.Frame)
    assert Pose is Frame
    twist = Twist(PyKDL.Vector(0, 0, 0), PyKDL.Vector(1, 0, 0))
    assert isinstance(twist, PyKDL.Twist)


def test_joint_state_validate_vector_lengths():
    with pytest.raises(ValueError, match="shape"):
        JointState(("yaw", "pitch"), np.zeros(1), np.zeros(2))
    with pytest.raises(ValueError, match="unique"):
        JointState(("yaw", "yaw"), np.zeros(2), np.zeros(2))


def test_ik_result_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        IKResult(np.array([np.nan]), False)
