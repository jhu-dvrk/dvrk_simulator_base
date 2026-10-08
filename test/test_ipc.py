"""Exercise real socket framing, backpressure, and the process-neutral values."""

import json
import socket
import struct

import numpy as np
import pytest

from dvrk_simulator_base.cartesian_command import CartesianCommand
from dvrk_simulator_base.command_mailbox import CommandEnvelope, CommandMailboxes
from dvrk_simulator_base.ipc import MAX_FRAME_BYTES, UnixSocketEndpoint, decode_message, encode_message
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


def snapshot():
    joints = JointState(("yaw", "pitch"), [0.1, 0.2], [0.3, 0.4], [1.0, 2.0])
    pose = Pose([1, 2, 3], np.eye(3))
    return ArmSnapshot(7, 0.125, True, joints, joints, pose, pose,
                       Twist([1, 0, 0], [0, 0, 1]), 0.4, 0.5,
                       OperatingStateSnapshot("ENABLED", True, True), True)


def test_roundtrip_scene_and_cartesian_command():
    source = {"kind": "snapshots", "snapshots": {"PSM1": snapshot()}}
    result = decode_message(encode_message(source)[4:])["snapshots"]["PSM1"]
    assert result.sequence == 7 and result.simulation_time == 0.125
    assert result.operating_state.is_busy
    np.testing.assert_array_equal(result.measured_js.effort, [1, 2])
    np.testing.assert_array_equal(result.measured_cp_world.position, [1, 2, 3])
    assert not result.measured_js.position.flags.writeable
    command = CommandEnvelope("servo_cp", 12, 987654, CartesianCommand(result.measured_cp_world, "ECM_view"))
    decoded = decode_message(encode_message({"kind": "commands", "command": command})[4:])["command"]
    assert decoded.sequence == 12 and decoded.received_at_ns == 987654
    assert decoded.payload.frame_id == "ECM_view"
    np.testing.assert_array_equal(decoded.payload.pose.position, [1, 2, 3])


def test_fragmented_frames_and_multiple_messages():
    left, right = socket.socketpair()
    receiver = UnixSocketEndpoint(right)
    try:
        data = encode_message({"kind": "first"}) + encode_message({"kind": "second"})
        left.sendall(data[:2])
        assert receiver.poll() == []
        left.sendall(data[2:9])
        assert receiver.poll() == []
        left.sendall(data[9:])
        assert [message["kind"] for message in receiver.poll()] == ["first", "second"]
    finally:
        left.close()
        receiver.close()


def test_coalesced_snapshots_preserve_reliable_events():
    left, right = socket.socketpair()
    sender, receiver = UnixSocketEndpoint(left), UnixSocketEndpoint(right)
    try:
        sender.send({"kind": "snapshots", "sequence": 1}, latest_key="scene")
        sender.send({"kind": "state_event", "busy": True})
        sender.send({"kind": "state_event", "busy": False})
        sender.send({"kind": "snapshots", "sequence": 2}, latest_key="scene")
        sender.poll()
        messages = receiver.poll()
        assert [message["kind"] for message in messages] == ["state_event", "state_event", "snapshots"]
        assert messages[-1]["sequence"] == 2
    finally:
        sender.close()
        receiver.close()


def test_partial_write_is_never_replaced_and_peer_cannot_block_sender():
    left, right = socket.socketpair()
    left.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
    sender, receiver = UnixSocketEndpoint(left), UnixSocketEndpoint(right)
    try:
        sender.send({"kind": "snapshots", "sequence": 1, "data": "x" * 100000}, latest_key="scene")
        sender.poll()  # No reader: forced partial write, nonblocking.
        assert sender.pending_bytes > 0
        sender.send({"kind": "snapshots", "sequence": 2}, latest_key="scene")
        sender.send({"kind": "snapshots", "sequence": 3}, latest_key="scene")
        messages = []
        for _ in range(1000):
            sender.poll()
            messages.extend(receiver.poll())
            if not sender.pending_bytes:
                break
        assert [message["sequence"] for message in messages] == [1, 3]
    finally:
        sender.close()
        receiver.close()


def test_reliable_queue_is_bounded():
    left, right = socket.socketpair()
    sender = UnixSocketEndpoint(left, max_pending_bytes=100)
    try:
        with pytest.raises(BufferError, match="stalled"):
            for _ in range(10):
                sender.send({"kind": "event"})
        assert sender.pending_bytes <= 100
    finally:
        sender.close()
        right.close()


@pytest.mark.parametrize("size", [0, MAX_FRAME_BYTES + 1])
def test_invalid_frame_length(size):
    left, right = socket.socketpair()
    receiver = UnixSocketEndpoint(right)
    try:
        left.sendall(struct.pack("!I", size))
        with pytest.raises(ValueError, match="frame size"):
            receiver.poll()
    finally:
        left.close()
        receiver.close()


def test_disconnect_and_truncated_frame():
    left, right = socket.socketpair()
    receiver = UnixSocketEndpoint(right)
    left.sendall(encode_message({"kind": "stopped"}))
    left.close()
    assert receiver.poll() == [{"kind": "stopped"}]
    assert receiver.peer_closed
    receiver.close()
    left, right = socket.socketpair()
    receiver = UnixSocketEndpoint(right)
    left.sendall(b"\x00\x00")
    left.close()
    with pytest.raises(ConnectionError, match="during a frame"):
        receiver.poll()
    receiver.close()


def test_invalid_version_unknown_type_and_nonfinite_data():
    with pytest.raises(ValueError, match="version"):
        decode_message(b'{"version":999,"message":{"kind":"hello"}}')
    with pytest.raises(ValueError, match="unknown IPC"):
        decode_message(json.dumps({"version": 1, "message": {"__type__": "arbitrary", "fields": {}}}).encode())
    with pytest.raises(ValueError):
        encode_message({"kind": "hello", "value": float("nan")})


def test_received_envelopes_retain_order_age_and_coalescing():
    mailbox = CommandMailboxes(discrete_capacity=1)
    first = CommandEnvelope("servo_jp", 10, 12345, [0.1])
    state = CommandEnvelope("state_command", 11, 12346, "disable")
    last = CommandEnvelope("move_jp", 12, 12347, [0.2])
    assert mailbox.submit_envelope(first)
    assert mailbox.submit_envelope(state)
    assert mailbox.submit_envelope(last)
    assert not mailbox.submit_envelope(CommandEnvelope("state_command", 13, 12348, "enable"))
    assert mailbox.drain() == (state, last)
    assert mailbox.counters.coalesced == 1 and mailbox.counters.rejected == 1
