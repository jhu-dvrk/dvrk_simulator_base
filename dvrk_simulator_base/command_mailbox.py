"""Bounded thread-safe command transfer from ROS callbacks to backends."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any, Callable


@dataclass(frozen=True)
class CommandEnvelope:
    channel: str
    sequence: int
    received_at_ns: int
    payload: Any


@dataclass(frozen=True)
class DeferredCommandPayload:
    """Convert a superseding command only after the mailbox selects it."""

    message: Any
    converter: Callable[[Any], Any]

    def resolve(self) -> Any:
        return self.converter(self.message)


@dataclass(frozen=True)
class MailboxCounters:
    received: int
    coalesced: int
    rejected: int
    drained: int


class CommandMailboxes:
    """Latest arm/jaw motion plus a bounded queue of ordered state commands."""

    _ARM_MOTION = frozenset((
        "servo_jp", "servo_cp", "move_jp", "move_cp", "move_cp_world",
    ))
    _JAW_MOTION = frozenset(("jaw/servo_jp", "jaw/move_jp"))
    _MOVE_CHANNELS = frozenset(("move_jp", "move_cp", "move_cp_world", "jaw/move_jp"))

    def __init__(self, discrete_capacity: int = 32) -> None:
        if discrete_capacity <= 0:
            raise ValueError("discrete_capacity must be positive")
        self._capacity = int(discrete_capacity)
        self._lock = threading.Lock()
        self._next_sequence = 0
        self._superseding: dict[str, CommandEnvelope] = {}
        self._discrete: deque[CommandEnvelope] = deque()
        self._received = 0
        self._coalesced = 0
        self._rejected = 0
        self._drained = 0

    def _envelope(self, channel: str, payload: Any) -> CommandEnvelope:
        envelope = CommandEnvelope(
            channel=str(channel),
            sequence=self._next_sequence,
            received_at_ns=time.monotonic_ns(),
            payload=payload,
        )
        self._next_sequence += 1
        self._received += 1
        return envelope

    def submit_servo(self, channel: str, payload: Any) -> CommandEnvelope:
        """Replace the pending arm or jaw motion, including a pending move."""
        with self._lock:
            envelope = self._envelope(channel, payload)
            key = (
                "arm" if channel in self._ARM_MOTION else
                "jaw" if channel in self._JAW_MOTION else channel
            )
            if key in self._superseding:
                self._coalesced += 1
            self._superseding[key] = envelope
            return envelope

    def submit_discrete(self, channel: str, payload: Any) -> CommandEnvelope | None:
        """Supersede moves; queue other commands in receive order."""
        if channel in self._MOVE_CHANNELS:
            return self.submit_servo(channel, payload)
        with self._lock:
            if len(self._discrete) >= self._capacity:
                self._received += 1
                self._rejected += 1
                return None
            envelope = self._envelope(channel, payload)
            self._discrete.append(envelope)
            return envelope

    def submit_envelope(self, envelope: CommandEnvelope) -> bool:
        """Receive an IPC command without replacing its sequence or receive time."""
        with self._lock:
            self._received += 1
            self._next_sequence = max(self._next_sequence, envelope.sequence + 1)
            if envelope.channel in self._ARM_MOTION or envelope.channel in self._JAW_MOTION:
                key = "arm" if envelope.channel in self._ARM_MOTION else "jaw"
                if key in self._superseding:
                    self._coalesced += 1
                self._superseding[key] = envelope
            else:
                if len(self._discrete) >= self._capacity:
                    self._rejected += 1
                    return False
                self._discrete.append(envelope)
            return True

    def drain(self) -> tuple[CommandEnvelope, ...]:
        """Atomically take all pending commands in global receive order."""
        with self._lock:
            commands = [*self._discrete, *self._superseding.values()]
            self._discrete.clear()
            self._superseding.clear()
            commands.sort(key=lambda command: command.sequence)
            self._drained += len(commands)
            return tuple(commands)

    @property
    def counters(self) -> MailboxCounters:
        with self._lock:
            return MailboxCounters(
                received=self._received,
                coalesced=self._coalesced,
                rejected=self._rejected,
                drained=self._drained,
            )
