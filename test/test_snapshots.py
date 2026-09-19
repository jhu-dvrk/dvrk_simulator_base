import numpy as np
import pytest

from crtk.snapshots import ArmSnapshot, OperatingStateSnapshot
from crtk.types import JointState, Pose, Twist


def make_snapshot(simulation_time=0.0):
    joints = JointState(("yaw",), np.zeros(1), np.zeros(1))
    pose = Pose(np.zeros(3), np.eye(3))
    return ArmSnapshot(
        sequence=0,
        simulation_time=simulation_time,
        valid=True,
        measured_js=joints,
        setpoint_js=joints,
        measured_cp_world=pose,
        setpoint_cp_world=pose,
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
        jaw_measured=0.0,
        jaw_setpoint=0.0,
        operating_state=OperatingStateSnapshot("ENABLED", True, False),
    )


def test_snapshot_uses_immutable_common_values():
    snapshot = make_snapshot()
    assert not snapshot.measured_js.position.flags.writeable
    assert not snapshot.measured_cp_world.position.flags.writeable


def test_snapshot_rejects_invalid_time():
    with pytest.raises(ValueError, match="simulation time"):
        make_snapshot(np.nan)
