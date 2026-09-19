"""Conversions from common immutable snapshots to ROS 2 messages."""

from __future__ import annotations

from crtk.rotations import rotation_to_quaternion_xyzw
from crtk.snapshots import OperatingStateSnapshot
from crtk.types import JointState, Pose, Twist


def joint_state_message(state: JointState, stamp, frame_id: str):
    from sensor_msgs.msg import JointState as JointStateMessage

    message = JointStateMessage()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.name = list(state.names)
    message.position = state.position.tolist()
    message.velocity = state.velocity.tolist()
    if state.effort is not None:
        message.effort = state.effort.tolist()
    return message


def pose_stamped_message(pose: Pose, stamp, frame_id: str):
    from geometry_msgs.msg import PoseStamped

    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
    quaternion = rotation_to_quaternion_xyzw(pose.orientation)
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = quaternion
    return message


def twist_stamped_message(twist: Twist, stamp, frame_id: str):
    from geometry_msgs.msg import TwistStamped

    message = TwistStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = twist.linear
    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = twist.angular
    return message


def operating_state_message(state: OperatingStateSnapshot, stamp, frame_id: str):
    from crtk_msgs.msg import OperatingState

    message = OperatingState()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.state = state.state
    message.is_homed = state.is_homed
    message.is_busy = state.is_busy
    return message


def string_stamped_message(value: str, stamp, frame_id: str):
    from crtk_msgs.msg import StringStamped

    message = StringStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.string = str(value)
    return message
