"""Backend-neutral simulation worker loop; no ROS or engine imports."""

from __future__ import annotations

import argparse
import os
import select
import signal
import socket
import time
import traceback

from .ipc import UnixSocketEndpoint


def run_worker(endpoint: UnixSocketEndpoint, factory) -> int:
    """Consume one acknowledged command batch per step; never wait for ROS."""
    runtime = None
    stopping = False
    failed = False
    pending_batch = None

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        start = None
        deadline = time.monotonic() + 30.0
        while start is None and not stopping:
            for message in endpoint.poll():
                if message["kind"] != "start" or start is not None:
                    raise ValueError("expected a single IPC startup message")
                start = message
            if endpoint.peer_closed:
                return 0
            if time.monotonic() > deadline:
                raise TimeoutError("ROS process did not send startup configuration")
            if start is None:
                select.select([endpoint.socket], [], [], 0.01)
        if stopping:
            return 0
        runtime, commands = factory(start)
        initial = runtime.initialize()
        last_states = {name: snapshot.operating_state for name, snapshot in initial.items()}
        endpoint.send({"kind": "ready", "pid": os.getpid(), "snapshots": initial, "captured_at_ns": time.monotonic_ns()})
        period = 1.0 / runtime.options.simulation_rate_hz
        next_step = time.monotonic()
        next_metrics = next_step + 1.0
        metrics_started_at = next_step
        metrics_steps = 0
        timeout = os.environ.get("DVRK_SIMULATOR_TEST_TIMEOUT")
        deadline = None if timeout is None else next_step + float(timeout)
        while not stopping and runtime.is_connected():
            for message in endpoint.poll():
                kind = message["kind"]
                if kind == "shutdown":
                    stopping = True
                elif kind == "commands":
                    if pending_batch is not None:
                        raise ValueError("more than one command batch in flight")
                    pending_batch = message["batch"]
                    for name, envelopes in message["commands"].items():
                        for envelope in envelopes:
                            if not commands[name].submit_envelope(envelope):
                                endpoint.send({"kind": "warning", "arm": name,
                                               "text": f"rejected {envelope.channel}: command queue is full"})
                else:
                    raise ValueError(f"unexpected worker message: {kind}")
            if endpoint.peer_closed or stopping:
                break
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            if now < next_step:
                select.select([endpoint.socket], [endpoint.socket] if endpoint.pending_bytes else [], [],
                              min(next_step - now, 0.01))
                continue
            snapshots = runtime.step()
            captured_at_ns = time.monotonic_ns()
            metrics_steps += 1
            if pending_batch is not None:
                endpoint.send({"kind": "commands_ack", "batch": pending_batch})
                pending_batch = None
            for name, snapshot in snapshots.items():
                if snapshot.operating_state_event or snapshot.operating_state != last_states[name]:
                    endpoint.send({"kind": "state_event", "arm": name,
                                   "state": snapshot.operating_state,
                                   "simulation_time": snapshot.simulation_time})
                    last_states[name] = snapshot.operating_state
            for name, text in runtime.command_warnings:
                endpoint.send({"kind": "warning", "arm": name, "text": text})
            runtime.command_warnings.clear()
            endpoint.send({"kind": "snapshots", "snapshots": snapshots, "captured_at_ns": captured_at_ns}, latest_key="scene")
            if now >= next_metrics:
                metrics = {"camera_hz": runtime.take_camera_rate_hz()}
                metrics["simulation_hz"] = metrics_steps / max(time.monotonic() - metrics_started_at, 1e-6)
                endpoint.send({"kind": "metrics", "metrics": metrics}, latest_key="metrics")
                metrics_steps = 0
                metrics_started_at = time.monotonic()
                next_metrics = metrics_started_at + 1.0
            next_step = max(next_step + period, time.monotonic())
    except Exception as error:
        failed = True
        traceback.print_exc()
        try:
            endpoint.send({"kind": "error", "text": str(error), "traceback": traceback.format_exc()})
        except (OSError, BufferError, ValueError):
            pass
    finally:
        if runtime is not None:
            try:
                runtime.shutdown()
            except Exception:
                failed = True
                traceback.print_exc()
        try:
            if not endpoint.peer_closed:
                endpoint.send({"kind": "stopped", "failed": failed})
                deadline = time.monotonic() + 1.0
                while endpoint.pending_bytes and not endpoint.peer_closed and time.monotonic() < deadline:
                    endpoint.poll()
                    select.select([], [endpoint.socket], [], 0.01)
        except (OSError, BufferError, ValueError):
            pass
        endpoint.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 1 if failed else 0


def worker_main(factory):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ipc-fd", required=True, type=int)
    options = parser.parse_args()
    return run_worker(UnixSocketEndpoint(socket.socket(fileno=options.ipc_fd)), factory)
