import numpy as np

from dvrk_simulator_base.cartesian_frames import (
    _VIEW_TO_OPTICAL_ROTATION,
    compose_pose,
    inverse_pose,
    relative_pose,
    relative_twist,
    view_pose_from_optical,
)
from crtk.types import Pose, Twist


def test_pose_compose_inverse_and_relative_round_trip():
    pose = Pose(np.array([1.0, 2.0, 3.0]), np.eye(3))
    identity = compose_pose(pose, inverse_pose(pose))
    np.testing.assert_allclose(identity.position, np.zeros(3))
    np.testing.assert_allclose(identity.orientation, np.eye(3))
    np.testing.assert_allclose(relative_pose(pose, pose).position, np.zeros(3))


def test_dvrk_view_axes_are_derived_from_ecm_optical_axes():
    view = view_pose_from_optical(Pose(np.zeros(3), np.eye(3)))
    np.testing.assert_allclose(view.orientation, _VIEW_TO_OPTICAL_ROTATION)
    np.testing.assert_allclose(view.orientation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(view.orientation @ [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(view.orientation @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])


def test_relative_twist_accounts_for_moving_reference():
    pose = Pose(np.array([1.0, 0.0, 0.0]), np.eye(3))
    result = relative_twist(
        pose,
        Twist(np.array([2.0, 0.0, 0.0]), np.zeros(3)),
        Pose(np.zeros(3), np.eye(3)),
        Twist(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    )
    np.testing.assert_allclose(result.linear, [1.0, -1.0, 0.0])
    np.testing.assert_allclose(result.angular, [0.0, 0.0, -1.0])
