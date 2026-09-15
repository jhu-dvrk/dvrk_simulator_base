"""Bounded thread-safe command transfer from ROS callbacks to backends."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any


@dataclass(frozen=True)
class CommandEnvelope:
    channel: str
    sequence: int
    received_at_ns: int
    payload: Any


@dataclass(frozen=True)
class MailboxCounters:
    received: int
    coalesced: int
    rejected: int
    drained: int


class CommandMailboxes:
    """Superseding servo slots plus one bounded ordered command queue."""

    def __init__(self, discrete_capacity: int = 32) -> None:
        if discrete_capacity <= 0:
            raise ValueError("discrete_capacity must be positive")
        self._capacity = int(discrete_capacity)
        self._lock = threading.Lock()
        self._next_sequence = 0
        self._servo: dict[str, CommandEnvelope] = {}
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
        """Replace any unprocessed servo command for ``channel``."""
        with self._lock:
            envelope = self._envelope(channel, payload)
            if channel in self._servo:
                self._coalesced += 1
            self._servo[channel] = envelope
            return envelope

    def submit_discrete(self, channel: str, payload: Any) -> CommandEnvelope | None:
        """Append an ordered command, or return ``None`` when the queue is full."""
        with self._lock:
            if len(self._discrete) >= self._capacity:
                self._received += 1
                self._rejected += 1
                return None
            envelope = self._envelope(channel, payload)
            self._discrete.append(envelope)
            return envelope

    def drain(self) -> tuple[CommandEnvelope, ...]:
        """Atomically take all pending commands in global receive order."""
        with self._lock:
            commands = [*self._discrete, *self._servo.values()]
            self._discrete.clear()
            self._servo.clear()
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
