"""Frontend supervision of a backend process over a private Unix socket."""

from __future__ import annotations

import socket
import subprocess
import time

from .ipc import UnixSocketEndpoint

class SimulationProcessError(RuntimeError):
    """The backend failed or violated the IPC session contract."""


class SimulationProcess:
    """Single ROS-thread owner of a separately launched simulation interpreter."""

    def __init__(self, python, worker_module: str, start: dict):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        self.endpoint = UnixSocketEndpoint(parent)
        self.process = None
        self.ready = False
        self.stopped = False
        self.worker_pid = None
        self._batch = 0
        self._pending_batch = None
        self._metrics = {}
        self._last_received_at = time.monotonic()
        self._snapshot_at_ns = None
        try:
            self.process = subprocess.Popen(
                [str(python), "-m", worker_module, "--ipc-fd", str(child.fileno())],
                pass_fds=(child.fileno(),),
            )
            self.endpoint.send({"kind": "start", **start})
        except BaseException:
            self.endpoint.close()
            if self.process is not None:
                self.process.terminate()
                self.process.wait()
            raise
        finally:
            child.close()

    def poll(self, node):
        messages = self.endpoint.poll()
        for message in messages:
            self._last_received_at = time.monotonic()
            kind = message["kind"]
            if kind == "ready":
                if self.ready:
                    raise SimulationProcessError("duplicate simulation startup message")
                self.ready = True
                self.worker_pid = message["pid"]
                node.install_initial_snapshots(message["snapshots"])
                self._snapshot_at_ns = message["captured_at_ns"]
                node.get_logger().info(f"simulation process ready (pid {self.worker_pid})")
            elif kind == "snapshots":
                self._snapshot_at_ns = message["captured_at_ns"]
                node.accept_snapshots(message["snapshots"])
            elif kind == "state_event":
                node.arm_interfaces[message["arm"]].snapshots.add_event(message["state"])
            elif kind == "warning":
                node.arm_interfaces[message["arm"]]._publish_warning(message["text"])
            elif kind == "commands_ack":
                if message["batch"] != self._pending_batch:
                    raise SimulationProcessError("unexpected command acknowledgement")
                self._pending_batch = None
            elif kind == "metrics":
                self._metrics = message["metrics"]
            elif kind == "error":
                detail = message.get("traceback") or message["text"]
                raise SimulationProcessError(f"simulation process failed: {detail}")
            elif kind == "stopped":
                self.stopped = True
                if message["failed"]:
                    raise SimulationProcessError("simulation process failed during shutdown")
            else:
                raise SimulationProcessError(f"unexpected simulation message: {kind}")
        if self.stopped:
            return
        if self.endpoint.peer_closed:
            raise SimulationProcessError("simulation socket closed unexpectedly")
        # A quiet backend may be compiling kernels or blocked in rendering.
        # Bound both startup and runtime silence without inventing new state.
        limit = 180.0 if not self.ready else 30.0
        if time.monotonic() - self._last_received_at > limit:
            raise SimulationProcessError("simulation process stopped responding")
        if self.ready and self._pending_batch is None:
            commands = {name: interface.commands.drain() for name, interface in node.arm_interfaces.items()}
            commands = {name: values for name, values in commands.items() if values}
            if commands:
                self._batch += 1
                self.endpoint.send({"kind": "commands", "batch": self._batch, "commands": commands})
                self._pending_batch = self._batch

    @property
    def metrics(self) -> dict[str, float]:
        return {
            "simulation_hz": self._metrics.get("simulation_hz", 0.0),
            "camera_hz": self._metrics.get("camera_hz", 0.0),
            "snapshot_age_ms": (0.0 if self._snapshot_at_ns is None else
                                (time.monotonic_ns() - self._snapshot_at_ns) * 1e-6),
        }

    def shutdown(self):
        """Request cleanup, then reap the worker even after errors or signals."""
        try:
            if self.process.poll() is None:
                try:
                    self.endpoint.send({"kind": "shutdown"})
                    deadline = time.monotonic() + 3.0
                    while self.endpoint.pending_bytes and time.monotonic() < deadline:
                        self.endpoint.poll()
                        if self.endpoint.peer_closed:
                            break
                        time.sleep(0.005)
                    deadline = time.monotonic() + 5.0
                    while self.process.poll() is None and time.monotonic() < deadline:
                        self.endpoint.poll()
                        time.sleep(0.005)
                    if self.process.poll() is None:
                        raise subprocess.TimeoutExpired(self.process.args, 5.0)
                    self.process.wait()
                except (OSError, BufferError, ValueError, subprocess.TimeoutExpired):
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
            else:
                self.process.wait()
        finally:
            self.endpoint.close()
