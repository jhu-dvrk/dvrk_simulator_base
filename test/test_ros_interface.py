import numpy as np
import PyKDL

from dvrk_simulator_base.ros_interface import (
    LatestSnapshot,
    _relative_twist,
    _VIEW_TO_OPTICAL_FRAME,
)
from dvrk.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk.types import JointState


def _snapshot(state, event=False):
    joints = JointState(("yaw",), [0.0], [0.0])
    return ArmSnapshot(
        sequence=0, simulation_time=0.0, valid=True, measured_js=joints,
        setpoint_js=joints, measured_cp_world=PyKDL.Frame(),
        setpoint_cp_world=PyKDL.Frame(),
        measured_cv_world=PyKDL.Twist(),
        jaw_measured=None, jaw_setpoint=None,
        operating_state=OperatingStateSnapshot(state, True, False),
        operating_state_event=event,
    )


def test_latest_snapshot_preserves_state_edges_and_consumes_them_once():
    snapshots = LatestSnapshot()
    initial = _snapshot("ENABLED")
    snapshots.set_initial(initial)
    snapshots.set(_snapshot("PAUSED"))

    value, events = snapshots.get_with_events()
    assert value is not None
    assert value.operating_state.state == "PAUSED"
    assert [event.state for event in events] == ["PAUSED"]
    assert snapshots.get_with_events()[1] == ()


def test_dvrk_view_axes_are_derived_from_ecm_optical_axes():
    view = PyKDL.Frame() * _VIEW_TO_OPTICAL_FRAME
    ex = view.M * PyKDL.Vector(1.0, 0.0, 0.0)
    ey = view.M * PyKDL.Vector(0.0, 1.0, 0.0)
    ez = view.M * PyKDL.Vector(0.0, 0.0, 1.0)
    np.testing.assert_allclose([ex[0], ex[1], ex[2]], [0.0, 1.0, 0.0])
    np.testing.assert_allclose([ey[0], ey[1], ey[2]], [0.0, 0.0, 1.0])
    np.testing.assert_allclose([ez[0], ez[1], ez[2]], [1.0, 0.0, 0.0])


def test_relative_twist_accounts_for_moving_reference():
    pose = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(1.0, 0.0, 0.0))
    result = _relative_twist(
        pose,
        PyKDL.Twist(PyKDL.Vector(2.0, 0.0, 0.0), PyKDL.Vector(0.0, 0.0, 0.0)),
        PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0.0, 0.0, 0.0)),
        PyKDL.Twist(PyKDL.Vector(1.0, 0.0, 0.0), PyKDL.Vector(0.0, 0.0, 1.0)),
    )
    np.testing.assert_allclose([result.vel[0], result.vel[1], result.vel[2]], [1.0, -1.0, 0.0])
    np.testing.assert_allclose([result.rot[0], result.rot[1], result.rot[2]], [0.0, 0.0, -1.0])