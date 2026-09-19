import numpy as np

from dvrk_simulator_base.ros_interface import LatestSnapshot
from crtk.snapshots import ArmSnapshot, OperatingStateSnapshot
from crtk.types import JointState, Pose, Twist


def _snapshot(state, event=False):
    joints = JointState(("yaw",), [0.0], [0.0])
    return ArmSnapshot(
        sequence=0, simulation_time=0.0, valid=True, measured_js=joints,
        setpoint_js=joints, measured_cp_world=Pose(np.zeros(3), np.eye(3)),
        setpoint_cp_world=Pose(np.zeros(3), np.eye(3)),
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
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