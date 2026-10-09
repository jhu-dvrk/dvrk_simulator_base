"""CRTK motion invariants shared by all simulator engines."""
from types import SimpleNamespace

import numpy as np
import pytest

from dvrk_simulator_base.arm_controller import ArmController
from dvrk_simulator_base.command_mailbox import CommandMailboxes


def controller():
    config = SimpleNamespace(home_position=(0.0,), raw={},
                             joints=(SimpleNamespace(lower=-1.0, upper=1.0, velocity=1.0),))
    return ArmController(config, CommandMailboxes())


def step(arm, now, ik=None):
    arm.advance_commands(now, ik or (lambda target, seed: None), lambda target: target)


def test_arm_servo_does_not_cancel_jaw_move():
    arm = controller()
    arm.commands.submit_discrete('jaw/move_jp', 0.4)
    step(arm, 0.0)
    arm.commands.submit_servo('servo_jp', [0.2])
    step(arm, 0.5)
    assert arm.joint_setpoint == pytest.approx([0.2])
    assert arm.jaw_setpoint == pytest.approx(0.2)
    assert arm.jaw_trajectory is not None
    step(arm, 1.0)
    assert arm.jaw_setpoint == pytest.approx(0.4)
    assert arm.jaw_trajectory is None
    assert arm.jaw_velocity == 0.0


def test_pause_cancels_both_moves_and_rejects_motion_until_resume():
    arm = controller()
    arm.commands.submit_discrete('move_jp', [1.0])
    arm.commands.submit_discrete('jaw/move_jp', 0.4)
    step(arm, 0.0)
    step(arm, 0.25)
    arm.commands.submit_discrete('state_command', 'pause')
    arm.commands.submit_discrete('move_jp', [-1.0])
    step(arm, 0.5)
    assert arm.move_failure_pending
    assert arm.joint_setpoint == pytest.approx([0.25])
    assert arm.joint_trajectory is arm.jaw_trajectory is None
    assert arm.joint_velocity == pytest.approx([0.0])
    arm.commands.submit_discrete('state_command', 'resume')
    arm.commands.submit_servo('servo_jp', [-0.2])
    step(arm, 1.0)
    assert not arm.move_failure_pending
    assert arm.joint_setpoint == pytest.approx([-0.2])


def test_disable_freezes_rate_limited_engine_at_measured_position():
    arm = controller()
    arm.commands.submit_servo('servo_jp', [1.0])
    step(arm, 0.0)
    arm.commands.submit_discrete('state_command', 'disable')
    arm.advance_commands(0.1, None, None, measured_position=np.array([0.1]))
    assert arm.joint_setpoint == pytest.approx([0.1])


def test_failed_cartesian_move_retains_previous_target_and_reports_failure():
    arm = controller()
    arm.commands.submit_discrete('move_cp', object())
    step(arm, 0.0, lambda target, seed: SimpleNamespace(success=False, position=np.array([0.2])))
    assert arm.move_failure_pending
    assert arm.joint_setpoint == pytest.approx([0.0])
    assert 'IK failed' in arm.command_warnings[-1]
    step(arm, 0.1)
    assert not arm.move_failure_pending


@pytest.mark.parametrize('target', [[2.0], [np.nan], [[0.2]], []])
def test_invalid_joint_targets_do_not_change_setpoint(target):
    arm = controller()
    arm.commands.submit_discrete('move_jp', target)
    step(arm, 0.0)
    assert arm.move_failure_pending
    assert arm.joint_setpoint == pytest.approx([0.0])
