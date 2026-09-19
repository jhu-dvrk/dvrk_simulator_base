"""Conversions from common immutable snapshots to ROS 2 messages."""

from __future__ import annotations

import PyKDL

from dvrk.snapshots import OperatingStateSnapshot
from dvrk.types import JointState


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


def pose_stamped_message(pose: PyKDL.Frame, stamp, frame_id: str):
    from geometry_msgs.msg import PoseStamped

    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x = float(pose.p[0])
    message.pose.position.y = float(pose.p[1])
    message.pose.position.z = float(pose.p[2])
    qx, qy, qz, qw = pose.M.GetQuaternion()
    message.pose.orientation.x = float(qx)
    message.pose.orientation.y = float(qy)
    message.pose.orientation.z = float(qz)
    message.pose.orientation.w = float(qw)
    return message


def twist_stamped_message(twist: PyKDL.Twist, stamp, frame_id: str):
    from geometry_msgs.msg import TwistStamped

    message = TwistStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.twist.linear.x = float(twist.vel[0])
    message.twist.linear.y = float(twist.vel[1])
    message.twist.linear.z = float(twist.vel[2])
    message.twist.angular.x = float(twist.rot[0])
    message.twist.angular.y = float(twist.rot[1])
    message.twist.angular.z = float(twist.rot[2])
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
