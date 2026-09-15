import pytest

from dvrk_simulator_base.operating_state import CRTKOperatingState


def test_operating_state_transitions_and_homing_gate_motion():
    state = CRTKOperatingState()
    assert state.state == state.DISABLED
    assert state.is_homed
    assert not state.accepts_motion

    assert state.command("enable")[0]
    assert state.accepts_motion
    assert state.command("unhome")[0]
    assert not state.accepts_motion
    assert state.command("home")[0]
    assert state.command("pause")[0]
    assert not state.accepts_motion
    assert state.command("resume")[0]
    assert state.command("fault")[0]
    assert not state.command("enable")[0]
    assert state.command("clear_fault")[0]
    assert state.state == state.DISABLED


def test_invalid_initial_state_and_command_are_rejected():
    with pytest.raises(ValueError, match="initial"):
        CRTKOperatingState("UNKNOWN")
    assert not CRTKOperatingState().command("not-a-command")[0]
