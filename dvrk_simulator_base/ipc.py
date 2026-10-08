"""Versioned JSON messages over a bounded, nonblocking Unix stream socket.

An endpoint has one owner thread. Reliable messages stay ordered; an unsent
message with a latest_key can be replaced. Partially written frames are never
replaced. No ROS, simulator libraries, pickle, or shared Python objects cross
this boundary.
"""

from collections import deque
from dataclasses import fields, is_dataclass
import json
import socket
import struct

import numpy as np

from .cartesian_command import CartesianCommand
from .command_mailbox import CommandEnvelope
from .snapshots import ArmPublicationFrames, ArmSnapshot, OperatingStateSnapshot
from .types import JointState, Pose, Twist

VERSION = 1
MAX_FRAME_BYTES = 1024 * 1024
_TYPES = {cls.__name__: cls for cls in (
    CartesianCommand, CommandEnvelope, ArmPublicationFrames, ArmSnapshot, OperatingStateSnapshot,
    JointState, Pose, Twist,
)}


def _encode(value):
    if is_dataclass(value):
        if type(value).__name__ not in _TYPES or _TYPES[type(value).__name__] is not type(value):
            raise TypeError(f"unsupported IPC type: {type(value).__name__}")
        return {"__type__": type(value).__name__,
                "fields": {field.name: _encode(getattr(value, field.name)) for field in fields(value)}}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _encode(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_encode(item) for item in value]
    return value


def _decode(value):
    if isinstance(value, dict):
        if "__type__" in value:
            cls = _TYPES.get(value["__type__"])
            if cls is None:
                raise ValueError("unknown IPC value type")
            return cls(**{key: _decode(item) for key, item in value["fields"].items()})
        return {key: _decode(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_decode(item) for item in value]
    return value


def encode_message(message: dict) -> bytes:
    data = json.dumps({"version": VERSION, "message": _encode(message)},
                      separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_FRAME_BYTES:
        raise ValueError("IPC message exceeds maximum frame size")
    return struct.pack("!I", len(data)) + data


def _reject_nonfinite(value):
    raise ValueError(f"non-finite IPC value: {value}")


def decode_message(data: bytes) -> dict:
    document = json.loads(data, parse_constant=_reject_nonfinite)
    if document.get("version") != VERSION:
        raise ValueError("unsupported IPC protocol version")
    message = _decode(document["message"])
    if not isinstance(message, dict) or not isinstance(message.get("kind"), str):
        raise ValueError("IPC message requires a kind")
    return message


class UnixSocketEndpoint:
    """Bounded framed transport; poll never waits on the peer."""

    def __init__(self, sock: socket.socket, *, max_pending_bytes: int = 4 * MAX_FRAME_BYTES):
        if sock.family != socket.AF_UNIX or sock.type & socket.SOCK_STREAM != socket.SOCK_STREAM:
            raise ValueError("IPC requires a Unix stream socket")
        self.socket = sock
        sock.setblocking(False)
        self._outgoing = deque()
        self._active = b""
        self._offset = 0
        self._incoming = bytearray()
        self._max_pending_bytes = max_pending_bytes
        self.peer_closed = False

    @property
    def pending_bytes(self) -> int:
        return len(self._active) - self._offset + sum(len(data) for _, data in self._outgoing)

    def send(self, message: dict, *, latest_key: str | None = None) -> None:
        data = encode_message(message)
        replacement = None
        if latest_key is not None:
            replacement = next((index for index, (key, _) in enumerate(self._outgoing)
                                if key == latest_key), None)
        previous = 0 if replacement is None else len(self._outgoing[replacement][1])
        if self.pending_bytes - previous + len(data) > self._max_pending_bytes:
            raise BufferError("IPC peer is stalled: outbound buffer is full")
        if replacement is None:
            self._outgoing.append((latest_key, data))
        else:
            # Put the replacement after intervening reliable messages.
            del self._outgoing[replacement]
            self._outgoing.append((latest_key, data))

    def poll(self) -> list[dict]:
        messages = []
        # Bound per-call work even if a producer continuously sends.
        budget = MAX_FRAME_BYTES
        while budget > 0 and not self.peer_closed:
            try:
                chunk = self.socket.recv(min(65536, budget))
            except BlockingIOError:
                break
            if not chunk:
                self.peer_closed = True
                if self._incoming:
                    raise ConnectionError("IPC peer closed during a frame")
                break
            self._incoming.extend(chunk)
            budget -= len(chunk)
            while len(self._incoming) >= 4:
                size = struct.unpack("!I", self._incoming[:4])[0]
                if size == 0 or size > MAX_FRAME_BYTES:
                    raise ValueError("invalid IPC frame size")
                if len(self._incoming) < size + 4:
                    break
                messages.append(decode_message(bytes(self._incoming[4:size + 4])))
                del self._incoming[:size + 4]
        budget = MAX_FRAME_BYTES
        while budget > 0 and not self.peer_closed:
            if not self._active:
                if not self._outgoing:
                    break
                _, self._active = self._outgoing.popleft()
                self._offset = 0
            try:
                count = self.socket.send(self._active[self._offset:self._offset + budget])
            except BlockingIOError:
                break
            if count == 0:
                raise ConnectionError("IPC socket stopped accepting data")
            self._offset += count
            budget -= count
            if self._offset == len(self._active):
                self._active = b""
                self._offset = 0
        return messages

    def close(self) -> None:
        self.socket.close()
