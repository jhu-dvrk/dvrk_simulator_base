import numpy as np

from dvrk_simulator_base.ros_interface import LatestSnapshot
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


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
    snapshots.add_event(OperatingStateSnapshot("PAUSED", True, False))
    snapshots.set(_snapshot("PAUSED"))

    value, events = snapshots.get_with_events()
    assert value is not None
    assert value.operating_state.state == "PAUSED"
    assert [event.state for event in events] == ["PAUSED"]
    assert snapshots.get_with_events()[1] == ()

def test_explicit_ipc_events_are_not_duplicated_by_snapshots():
    from dvrk_simulator_base.snapshots import OperatingStateSnapshot
    snapshots = LatestSnapshot()
    snapshots.set_initial(_snapshot("ENABLED"))
    snapshots.add_event(OperatingStateSnapshot("PAUSED", True, False))
    snapshots.add_event(OperatingStateSnapshot("ENABLED", True, False))
    snapshots.set(_snapshot("ENABLED", event=True))
    _, events = snapshots.get_with_events()
    assert [event.state for event in events] == ["PAUSED", "ENABLED"]
    assert snapshots.get_with_events()[1] == ()


def test_ros_ipc_adapter_parses_frames_without_converting_them(monkeypatch):
    from types import SimpleNamespace
    from dvrk_simulator_base import ros_interface
    from dvrk_simulator_base.cartesian_command import CartesianCommand
    from dvrk_simulator_base.command_mailbox import CommandMailboxes
    from dvrk_simulator_base.ros_interface import ArmRosInterface
    from dvrk_simulator_base.types import Pose
    interface = ArmRosInterface.__new__(ArmRosInterface)
    interface.commands = CommandMailboxes()
    interface._publish_warning = lambda text: None
    pose = Pose([1, 2, 3], np.eye(3))
    monkeypatch.setattr(ros_interface, "pose_from_message", lambda message: pose)
    interface._cartesian_command(SimpleNamespace(header=SimpleNamespace(frame_id="ECM_view")), "servo_cp")
    payload = interface.commands.drain()[0].payload
    assert isinstance(payload, CartesianCommand)
    assert payload.pose is pose and payload.frame_id == "ECM_view"


def test_ros_publication_uses_simulation_frames_without_conversion(monkeypatch):
    from dataclasses import replace
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from dvrk_simulator_base import ros_interface
    from dvrk_simulator_base.publication_frames import with_publication_frames
    from dvrk_simulator_base.ros_interface import ArmRosInterface

    config = SimpleNamespace(name="PSM1", type="PSM", base_frame="PSM1_base", parent_frame="world",
                             base_position=[1, 0, 0], base_orientation_xyzw=[0, 0, 0, 1])
    state = replace(_snapshot("ENABLED"), measured_cp_world=Pose([2, 0, 0], np.eye(3)))
    state = with_publication_frames({"PSM1": state}, [config])["PSM1"]
    interface = ArmRosInterface.__new__(ArmRosInterface)
    interface.config, interface.frame_id = config, "world"
    interface.snapshots = LatestSnapshot()
    interface.snapshots.set_initial(state)
    interface.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
    published = {}
    for name in ("measured_js", "setpoint_js", "measured_cp", "setpoint_cp", "measured_cv",
                 "local_measured_cp", "local_setpoint_cp"):
        setattr(interface, name, SimpleNamespace(publish=lambda value, key=name: published.__setitem__(key, value)))

    assert not hasattr(ros_interface, "relative_pose")
    interface.publish_latest()
    assert published["measured_cp"].pose.position.x == 2
    assert published["local_measured_cp"].pose.position.x == 1
    assert published["local_measured_cp"].header.frame_id == "PSM1_base"
