from types import SimpleNamespace

import numpy as np
import pytest

from dvrk_simulator_base.cartesian_command import CartesianCommand, resolve_cartesian_command
from dvrk_simulator_base.types import Pose


@pytest.fixture
def config():
    return SimpleNamespace(base_frame="PSM1_base", parent_frame="world",
                           base_position=[1, 2, 3], base_orientation_xyzw=[0, 0, 0, 1])


def test_world_base_and_default_without_ecm(config):
    pose = Pose([0.1, 0.2, 0.3], np.eye(3))
    assert resolve_cartesian_command(CartesianCommand(pose, "world"), config) is pose
    assert resolve_cartesian_command(CartesianCommand(pose), config) is pose
    base = resolve_cartesian_command(CartesianCommand(pose, "PSM1_base"), config)
    np.testing.assert_allclose(base.position, [1.1, 2.2, 3.3])


def test_moving_ecm_is_resolved_at_consumption(config):
    command = CartesianCommand(Pose([0.1, 0.2, 0.3], np.eye(3)), "ECM_view")
    first = resolve_cartesian_command(command, config, Pose([1, 0, 0], np.eye(3)), has_ecm=True)
    second = resolve_cartesian_command(command, config, Pose([2, 0, 0], np.eye(3)), has_ecm=True)
    np.testing.assert_allclose(first.position, [1.3, 0.1, 0.2])
    np.testing.assert_allclose(second.position - first.position, [1, 0, 0])
    default = resolve_cartesian_command(CartesianCommand(command.pose), config,
                                        Pose([2, 0, 0], np.eye(3)), has_ecm=True)
    np.testing.assert_allclose(default.position, second.position)


def test_missing_ecm_and_unknown_frames(config):
    pose = Pose([0, 0, 0], np.eye(3))
    with pytest.raises(ValueError, match="unavailable"):
        resolve_cartesian_command(CartesianCommand(pose, "ECM_view"), config, has_ecm=True)
    for has_ecm in (False, True):
        with pytest.raises(ValueError, match="unsupported"):
            resolve_cartesian_command(CartesianCommand(pose, "unknown"), config, has_ecm=has_ecm)
