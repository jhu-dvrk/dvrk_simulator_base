import numpy as np
from dvrk_simulator_base.cartesian_frames import optical_pose_from_view, view_pose_from_optical
from dvrk_simulator_base.types import Pose


def test_optical_and_controller_axes_and_round_trip():
    view = Pose([.1, .2, .3], np.eye(3))
    optical = optical_pose_from_view(view)
    np.testing.assert_allclose(optical.orientation, np.diag([-1., -1., 1.]))
    np.testing.assert_allclose(optical.position, view.position)
    restored = view_pose_from_optical(optical)
    np.testing.assert_allclose(restored.orientation, view.orientation)
