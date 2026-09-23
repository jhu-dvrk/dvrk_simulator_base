import numpy as np
import pytest

from dvrk_simulator_base.rotations import (
    quaternion_matrix_xyzw,
    rotation_to_quaternion_xyzw,
)


def test_quaternion_round_trip_uses_xyzw_order():
    quaternion = np.array([0.2, -0.3, 0.1, 0.9])
    quaternion /= np.linalg.norm(quaternion)
    result = np.asarray(rotation_to_quaternion_xyzw(quaternion_matrix_xyzw(quaternion)))
    if np.dot(result, quaternion) < 0.0:
        result = -result
    np.testing.assert_allclose(result, quaternion, atol=1e-12)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError, match="zero"):
        quaternion_matrix_xyzw(np.zeros(4))


def test_non_rotation_matrix_is_rejected():
    with pytest.raises(ValueError, match="orthonormal"):
        rotation_to_quaternion_xyzw(np.zeros((3, 3)))
