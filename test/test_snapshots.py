import numpy as np
import pytest
import PyKDL

from dvrk.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk.types import JointState


def make_snapshot(simulation_time=0.0):
    joints = JointState(("yaw",), np.zeros(1), np.zeros(1))
    pose = PyKDL.Frame()
    return ArmSnapshot(
        sequence=0,
        simulation_time=simulation_time,
        valid=True,
        measured_js=joints,
        setpoint_js=joints,
        measured_cp_world=pose,
        setpoint_cp_world=pose,
        measured_cv_world=PyKDL.Twist(),
        jaw_measured=0.0,
        jaw_setpoint=0.0,
        operating_state=OperatingStateSnapshot("ENABLED", True, False),
    )


def test_snapshot_uses_immutable_common_values():
    snapshot = make_snapshot()
    assert not snapshot.measured_js.position.flags.writeable
    assert isinstance(snapshot.measured_cp_world, PyKDL.Frame)
    assert isinstance(snapshot.measured_cv_world, PyKDL.Twist)


def test_snapshot_rejects_invalid_time():
    with pytest.raises(ValueError, match="simulation time"):
        make_snapshot(np.nan)
