"""Backend-neutral ROS 2 CRTK arm adapter."""

from __future__ import annotations

from collections import deque
import threading

from .cartesian_frames import compose_pose, relative_pose, relative_twist, view_pose_from_optical
from .command_mailbox import CommandMailboxes
from .command_validation import jaw_position_from_message, joint_positions_from_message, pose_from_message
from crtk.config import RobotConfig
from .ros_messages import joint_state_message, operating_state_message, pose_stamped_message, string_stamped_message, twist_stamped_message
from .ros_qos import transient_local_event_qos, transient_local_latched_qos
from crtk.rotations import quaternion_matrix_xyzw
from crtk.snapshots import ArmSnapshot, OperatingStateSnapshot
from crtk.types import JointState, Pose


class LatestSnapshot:
    """Thread-safe snapshot handoff from a simulator owner thread to ROS."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: ArmSnapshot | None = None
        self._last_operating_state: OperatingStateSnapshot | None = None
        self._operating_state_events: deque[OperatingStateSnapshot] = deque(maxlen=32)

    def set_initial(self, snapshot: ArmSnapshot) -> None:
        with self._lock:
            self._value = snapshot
            self._last_operating_state = snapshot.operating_state
            self._operating_state_events.clear()

    def set(self, snapshot: ArmSnapshot) -> None:
        with self._lock:
            if snapshot.operating_state_event or snapshot.operating_state != self._last_operating_state:
                self._operating_state_events.append(snapshot.operating_state)
                self._last_operating_state = snapshot.operating_state
            self._value = snapshot

    def peek(self) -> ArmSnapshot | None:
        with self._lock:
            return self._value

    def get_with_events(self) -> tuple[ArmSnapshot | None, tuple[OperatingStateSnapshot, ...]]:
        with self._lock:
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

        self.node, self.config, self.ecm_interface = node, config, ecm_interface
        self.commands = CommandMailboxes(command_queue_capacity)
        self.snapshots = LatestSnapshot()
        self._last_event_stamp_ns = -1
        self.base_pose = Pose(config.base_position, quaternion_matrix_xyzw(config.base_orientation_xyzw))
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

    def _world_view_pose(self) -> Pose | None:
        snapshot = None if self.ecm_interface is None else self.ecm_interface.snapshots.peek()
        return None if snapshot is None else view_pose_from_optical(snapshot.measured_cp_world)

    def _cartesian_command(self, message, channel: str) -> None:
        try:
            target = pose_from_message(message)
            frame_id = message.header.frame_id
            if frame_id == self.config.base_frame:
                target = compose_pose(self.base_pose, target)
            elif self.ecm_interface is not None and frame_id != self.config.parent_frame:
                if frame_id and frame_id != "ECM_view":
                    raise ValueError(f"unsupported frame {frame_id!r}")
                world_view = self._world_view_pose()
                if world_view is None:
                    raise ValueError("ECM state is unavailable")
                target = compose_pose(world_view, target)
            elif frame_id and frame_id != self.config.parent_frame:
                raise ValueError(f"unsupported frame {frame_id!r}")
        except (TypeError, ValueError, AttributeError) as error:
            self._publish_warning(f"rejected {channel}: {error}")
            return
        if channel == "servo_cp": self.commands.submit_servo(channel, target)
        elif self.commands.submit_discrete(channel, target) is None: self._publish_warning(f"rejected {channel}: command queue is full")

    def _servo_cp_callback(self, message) -> None: self._cartesian_command(message, "servo_cp")
    def _move_cp_callback(self, message) -> None: self._cartesian_command(message, "move_cp")
    def _state_command_callback(self, message) -> None:
        if self.commands.submit_discrete("state_command", message.string) is None: self._publish_warning("rejected state_command: command queue is full")

    def install_initial_snapshot(self, snapshot: ArmSnapshot) -> None:
        from std_msgs.msg import String
        self.snapshots.set_initial(snapshot); stamp = self._event_stamp()
        self.operating_state.publish(operating_state_message(snapshot.operating_state, stamp, self.frame_id))
        self.state.publish(string_stamped_message(snapshot.operating_state.state, stamp, self.frame_id))
        if self.tool_type is not None:
            message = String(); message.data = self.config.instrument or ""; self.tool_type.publish(message)

    def publish_latest(self) -> None:
        snapshot, events = self.snapshots.get_with_events()
        if snapshot is None: return
        stamp = self.node.get_clock().now().to_msg(); measured_pose = snapshot.measured_cp_world; setpoint_pose = snapshot.setpoint_cp_world; measured_twist = snapshot.measured_cv_world
        world_view = self._world_view_pose()
        if world_view is not None and self.ecm_interface is not None:
            ecm_snapshot = self.ecm_interface.snapshots.peek(); measured_pose = relative_pose(measured_pose, world_view); setpoint_pose = relative_pose(setpoint_pose, world_view)
            if ecm_snapshot is not None: measured_twist = relative_twist(snapshot.measured_cp_world, snapshot.measured_cv_world, world_view, ecm_snapshot.measured_cv_world)
        self.measured_js.publish(joint_state_message(snapshot.measured_js, stamp, self.frame_id)); self.setpoint_js.publish(joint_state_message(snapshot.setpoint_js, stamp, self.frame_id)); self.measured_cp.publish(pose_stamped_message(measured_pose, stamp, self.frame_id)); self.setpoint_cp.publish(pose_stamped_message(setpoint_pose, stamp, self.frame_id)); self.measured_cv.publish(twist_stamped_message(measured_twist, stamp, self.frame_id))
        for state in events:
            event_stamp = self._event_stamp(); self.operating_state.publish(operating_state_message(state, event_stamp, self.frame_id)); self.state.publish(string_stamped_message(state.state, event_stamp, self.frame_id))
        if self.config.type == "PSM":
            self.local_measured_cp.publish(pose_stamped_message(relative_pose(snapshot.measured_cp_world, self.base_pose), stamp, self.config.base_frame)); self.local_setpoint_cp.publish(pose_stamped_message(relative_pose(snapshot.setpoint_cp_world, self.base_pose), stamp, self.config.base_frame))
        if snapshot.jaw_measured is not None and self.jaw_measured_js is not None:
            measured = JointState(("jaw",), [snapshot.jaw_measured], [0.0]); value = snapshot.jaw_measured if snapshot.jaw_setpoint is None else snapshot.jaw_setpoint
            self.jaw_measured_js.publish(joint_state_message(measured, stamp, self.frame_id)); self.jaw_setpoint_js.publish(joint_state_message(JointState(("jaw",), [value], [0.0]), stamp, self.frame_id))