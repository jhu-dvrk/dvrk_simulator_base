import numpy as np

from builtin_interfaces.msg import Time

from dvrk_simulator_base.ros_messages import (
    joint_state_message,
    operating_state_message,
    pose_stamped_message,
)
from crtk.snapshots import OperatingStateSnapshot
from crtk.types import JointState, Pose


def test_joint_state_message_preserves_order_and_frame():
    message = joint_state_message(
        JointState(("yaw", "pitch"), np.array([0.1, 0.2]), np.zeros(2)),
        Time(sec=2, nanosec=3),
        "world",
    )
    assert message.header.frame_id == "world"
    assert message.header.stamp.sec == 2
    assert message.name == ["yaw", "pitch"]
    np.testing.assert_allclose(message.position, [0.1, 0.2])


def test_pose_and_operating_state_messages():
    stamp = Time()
    pose = pose_stamped_message(Pose(np.zeros(3), np.eye(3)), stamp, "world")
    assert pose.pose.orientation.w == 1.0
    state = operating_state_message(
        OperatingStateSnapshot("ENABLED", True, False), stamp, "world"
    )
    assert state.state == "ENABLED"
    assert state.is_homed
    assert not state.is_busy
