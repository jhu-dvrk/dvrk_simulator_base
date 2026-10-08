"""Backend-neutral ROS 2 CRTK arm adapter."""

from __future__ import annotations

from collections import deque

from .cartesian_command import CartesianCommand
from .command_mailbox import CommandMailboxes
from .command_validation import jaw_position_from_message, joint_positions_from_message, pose_from_message
from dvrk_arm_description import RobotConfig
from .ros_messages import joint_state_message, operating_state_message, pose_stamped_message, string_stamped_message, twist_stamped_message
from .ros_qos import transient_local_event_qos, transient_local_latched_qos
from .snapshots import ArmSnapshot
from .types import JointState


class LatestSnapshot:
    """Scene state and reliable events, owned by the ROS frontend thread."""

    def __init__(self):
        self._value = None
        self._operating_state_events = deque()

    def set_initial(self, snapshot):
        self._value = snapshot
        self._operating_state_events.clear()

    def set(self, snapshot):
        self._value = snapshot

    def add_event(self, state):
        if len(self._operating_state_events) >= 32:
            raise BufferError("operating-state event queue is full")
        self._operating_state_events.append(state)

    def peek(self):
        return self._value

    def get_with_events(self):
        events = tuple(self._operating_state_events)
        self._operating_state_events.clear()
        return self._value, events


class ArmRosInterface:
    """Expose one simulator arm through the common CRTK ROS graph."""

    def __init__(self, node, config: RobotConfig, command_queue_capacity: int,
                 ecm_interface: "ArmRosInterface | None" = None) -> None:
        from crtk_msgs.msg import OperatingState, StringStamped
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState as JointStateMessage
        from std_msgs.msg import String

        self.node, self.config = node, config
        self.commands = CommandMailboxes(command_queue_capacity)
        self.snapshots = LatestSnapshot()
        self._last_event_stamp_ns = -1
        self.frame_id = "ECM_view" if config.type == "PSM" and ecm_interface is not None else config.parent_frame
        prefix = f"/{config.name}"
        event_qos = transient_local_event_qos()
        self.measured_js = node.create_publisher(JointStateMessage, f"{prefix}/measured_js", 10)
        self.setpoint_js = node.create_publisher(JointStateMessage, f"{prefix}/setpoint_js", 10)
        self.measured_cp = node.create_publisher(PoseStamped, f"{prefix}/measured_cp", 10)
        self.setpoint_cp = node.create_publisher(PoseStamped, f"{prefix}/setpoint_cp", 10)
        self.measured_cv = node.create_publisher(TwistStamped, f"{prefix}/measured_cv", 10)
        self.operating_state = node.create_publisher(OperatingState, f"{prefix}/operating_state", event_qos)
        self.state = node.create_publisher(StringStamped, f"{prefix}/state", event_qos)
        self.info = node.create_publisher(StringStamped, f"{prefix}/info", 10)
        self.warning = node.create_publisher(StringStamped, f"{prefix}/warning", 10)
        self.error = node.create_publisher(StringStamped, f"{prefix}/error", 10)
        self.servo_jp = node.create_subscription(JointStateMessage, f"{prefix}/servo_jp", self._servo_jp_callback, 1)
        self.move_jp = node.create_subscription(JointStateMessage, f"{prefix}/move_jp", self._move_jp_callback, 10)
        self.servo_cp = node.create_subscription(PoseStamped, f"{prefix}/servo_cp", self._servo_cp_callback, 1)
        self.move_cp = node.create_subscription(PoseStamped, f"{prefix}/move_cp", self._move_cp_callback, 10)
        self.state_command = node.create_subscription(StringStamped, f"{prefix}/state_command", self._state_command_callback, 10)
        self.jaw_measured_js = self.jaw_setpoint_js = self.jaw_servo_jp = self.jaw_move_jp = self.tool_type = None
        self.local_measured_cp = self.local_setpoint_cp = None
        if config.type == "PSM":
            self.jaw_measured_js = node.create_publisher(JointStateMessage, f"{prefix}/jaw/measured_js", 10)
            self.jaw_setpoint_js = node.create_publisher(JointStateMessage, f"{prefix}/jaw/setpoint_js", 10)
            self.jaw_servo_jp = node.create_subscription(JointStateMessage, f"{prefix}/jaw/servo_jp", self._jaw_servo_jp_callback, 1)
            self.jaw_move_jp = node.create_subscription(JointStateMessage, f"{prefix}/jaw/move_jp", self._jaw_move_jp_callback, 10)
            self.tool_type = node.create_publisher(String, f"{prefix}/tool_type", transient_local_latched_qos())
            self.local_measured_cp = node.create_publisher(PoseStamped, f"{prefix}/local/measured_cp", 10)
            self.local_setpoint_cp = node.create_publisher(PoseStamped, f"{prefix}/local/setpoint_cp", 10)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.config.joints)

    def _publish_warning(self, value: str) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self.warning.publish(string_stamped_message(value, stamp, self.frame_id))
        self.node.get_logger().warning(f"{self.config.name}: {value}")

    def _event_stamp(self):
        stamp = self.node.get_clock().now().to_msg()
        value = max(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec), self._last_event_stamp_ns + 1)
        self._last_event_stamp_ns = value
        stamp.sec, stamp.nanosec = divmod(value, 1_000_000_000)
        return stamp

    def _submit(self, message, channel: str, parser) -> None:
        try:
            target = parser(message)
        except (TypeError, ValueError, AttributeError) as error:
            self._publish_warning(f"rejected {channel}: {error}")
            return
        if channel.startswith("servo") or channel == "jaw/servo_jp":
            self.commands.submit_servo(channel, target)
        elif self.commands.submit_discrete(channel, target) is None:
            self._publish_warning(f"rejected {channel}: command queue is full")

    def _servo_jp_callback(self, message) -> None:
        self._submit(message, "servo_jp", lambda item: joint_positions_from_message(item, self.joint_names))

    def _move_jp_callback(self, message) -> None:
        self._submit(message, "move_jp", lambda item: joint_positions_from_message(item, self.joint_names))

    def _jaw_servo_jp_callback(self, message) -> None:
        self._submit(message, "jaw/servo_jp", jaw_position_from_message)

    def _jaw_move_jp_callback(self, message) -> None:
        self._submit(message, "jaw/move_jp", jaw_position_from_message)

    def _cartesian_command(self, message, channel):
        self._submit(message, channel, lambda item: CartesianCommand(
            pose_from_message(item), str(item.header.frame_id),
        ))

    def _servo_cp_callback(self, message) -> None:
        self._cartesian_command(message, "servo_cp")

    def _move_cp_callback(self, message) -> None:
        self._cartesian_command(message, "move_cp")

    def _state_command_callback(self, message) -> None:
        if self.commands.submit_discrete("state_command", message.string) is None:
            self._publish_warning("rejected state_command: command queue is full")

    def install_initial_snapshot(self, snapshot: ArmSnapshot) -> None:
        from std_msgs.msg import String

        self.snapshots.set_initial(snapshot)
        stamp = self._event_stamp()
        self.operating_state.publish(operating_state_message(snapshot.operating_state, stamp, self.frame_id))
        self.state.publish(string_stamped_message(snapshot.operating_state.state, stamp, self.frame_id))
        if self.tool_type is not None:
            message = String()
            message.data = self.config.instrument or ""
            self.tool_type.publish(message)

    def publish_latest(self) -> None:
        snapshot, events = self.snapshots.get_with_events()
        if snapshot is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        frames = snapshot.publication_frames
        if frames is None or frames.frame_id != self.frame_id:
            raise ValueError("simulation snapshot lacks the configured publication frames")
        self.measured_js.publish(joint_state_message(snapshot.measured_js, stamp, self.frame_id))
        self.setpoint_js.publish(joint_state_message(snapshot.setpoint_js, stamp, self.frame_id))
        self.measured_cp.publish(pose_stamped_message(frames.measured_cp, stamp, self.frame_id))
        self.setpoint_cp.publish(pose_stamped_message(frames.setpoint_cp, stamp, self.frame_id))
        self.measured_cv.publish(twist_stamped_message(frames.measured_cv, stamp, self.frame_id))
        for state in events:
            event_stamp = self._event_stamp()
            self.operating_state.publish(operating_state_message(state, event_stamp, self.frame_id))
            self.state.publish(string_stamped_message(state.state, event_stamp, self.frame_id))
        if self.config.type == "PSM":
            self.local_measured_cp.publish(
                pose_stamped_message(frames.local_measured_cp, stamp, self.config.base_frame)
            )
            self.local_setpoint_cp.publish(
                pose_stamped_message(frames.local_setpoint_cp, stamp, self.config.base_frame)
            )
        if snapshot.jaw_measured is not None and self.jaw_measured_js is not None:
            measured = JointState(("jaw",), [snapshot.jaw_measured], [0.0])
            value = snapshot.jaw_measured if snapshot.jaw_setpoint is None else snapshot.jaw_setpoint
            setpoint = JointState(("jaw",), [value], [0.0])
            self.jaw_measured_js.publish(joint_state_message(measured, stamp, self.frame_id))
            self.jaw_setpoint_js.publish(joint_state_message(setpoint, stamp, self.frame_id))
