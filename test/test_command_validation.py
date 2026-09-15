import numpy as np
import pytest

from dvrk_simulator_base.command_validation import (
    jaw_position_from_message,
    joint_positions_from_message,
    pose_from_message,
)


class JointMessage:
    def __init__(self, name=(), position=()):
        self.name = list(name)
        self.position = list(position)


def test_named_joint_values_are_reordered():
    message = JointMessage(("pitch", "yaw"), (0.2, 0.1))
    np.testing.assert_allclose(
        joint_positions_from_message(message, ("yaw", "pitch")), [0.1, 0.2]
    )


def test_cisst_numeric_names_retain_vector_order():
    message = JointMessage(("0", "1"), (0.1, 0.2))
    np.testing.assert_allclose(
        joint_positions_from_message(message, ("yaw", "pitch")), [0.1, 0.2]
    )


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
def test_non_finite_joint_and_jaw_commands_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        joint_positions_from_message(JointMessage(position=(value,)), ("jaw",))
    with pytest.raises(ValueError, match="finite"):
        jaw_position_from_message(JointMessage(position=(value,)))


def test_duplicate_names_and_multi_value_jaw_are_rejected():
    with pytest.raises(ValueError):
        joint_positions_from_message(
            JointMessage(("yaw", "yaw"), (0.1, 0.2)), ("yaw", "pitch")
        )
    with pytest.raises(ValueError):
        jaw_position_from_message(JointMessage(position=(0.1, 0.2)))


def test_pose_message_is_validated_and_converted():
    class Value:
        pass

    message = Value()
    message.pose = Value()
    message.pose.position = Value()
    message.pose.orientation = Value()
    message.pose.position.x = 0.1
    message.pose.position.y = 0.2
    message.pose.position.z = 0.3
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 2.0

    pose = pose_from_message(message)
    np.testing.assert_allclose(pose.position, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(pose.orientation, np.eye(3))

    message.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="cannot be zero"):
        pose_from_message(message)
