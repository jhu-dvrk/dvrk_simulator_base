"""Pure-Python CRTK Cartesian frame conversions."""

from __future__ import annotations

import numpy as np

from .types import Pose, Twist


# The dVRK optical convention is +X forward, +Y left, +Z up. Teleoperation
# view coordinates use X left, Y up, Z away from the operator. This maps view
# coordinates into the ECM optical frame and is independent of a simulator.
VIEW_TO_OPTICAL_ROTATION = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def compose_pose(first: Pose, second: Pose) -> Pose:
    """Compose two poses represented as rotation/translation matrices."""
    return Pose(
        first.position + first.orientation @ second.position,
        first.orientation @ second.orientation,
    )


def inverse_pose(pose: Pose) -> Pose:
    """Return the inverse of a rigid pose."""
    rotation = pose.orientation.T
    return Pose(-rotation @ pose.position, rotation)


def relative_pose(pose: Pose, reference: Pose) -> Pose:
    """Express ``pose`` in the coordinate frame represented by ``reference``."""
    return compose_pose(inverse_pose(reference), pose)


def view_pose_from_optical(optical_pose: Pose) -> Pose:
    """Return the dVRK view pose derived from the current ECM optical FK."""
    return compose_pose(
        optical_pose, Pose(np.zeros(3), VIEW_TO_OPTICAL_ROTATION)
    )



def relative_twist(pose: Pose, twist: Twist, reference: Pose,
                   reference_twist: Twist) -> Twist:
    """Express a world twist in a moving reference frame."""
    delta = pose.position - reference.position
    linear = twist.linear - reference_twist.linear - np.cross(reference_twist.angular, delta)
    angular = twist.angular - reference_twist.angular
    rotation = reference.orientation.T
    return Twist(rotation @ linear, rotation @ angular)
