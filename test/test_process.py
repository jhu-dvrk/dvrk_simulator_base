"""Run the actual IPC worker loop in a fresh interpreter with a small backend."""

from dataclasses import replace
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dvrk_simulator_base import process as ipc_runtime
from dvrk_simulator_base.process import SimulationProcessError
from dvrk_simulator_base.cartesian_command import CartesianCommand, resolve_cartesian_command
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.publication_frames import with_publication_frames
from dvrk_simulator_base.ros_interface import LatestSnapshot
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


class FakeRuntime:
    def __init__(self, commands):
        assert "rclpy" not in sys.modules, "simulation interpreter must not load ROS"
        self.commands = commands
        self.options = SimpleNamespace(simulation_rate_hz=200.0)
        self.command_warnings = []
        self.configs = [SimpleNamespace(name=name, type=kind, base_frame=name + "_base", parent_frame="world",
                                       base_position=[0, 0, 0], base_orientation_xyzw=[0, 0, 0, 1])
                        for name, kind in (("ECM", "ECM"), ("PSM1", "PSM"))]
        self.states = {}
        self.sequence = 0
        for config in self.configs:
            joints = JointState(("yaw",), [0], [0])
            pose = Pose([0, 0, 0], np.eye(3))
            self.states[config.name] = ArmSnapshot(0, 0.0, True, joints, joints, pose, pose,
                Twist([0, 0, 0], [0, 0, 0]), None, None, OperatingStateSnapshot("ENABLED", True, False))

    def initialize(self):
        return with_publication_frames(self.states, self.configs)

    def is_connected(self):
        return True

    def step(self):
        self.sequence += 1
        previous_ecm = self.states["ECM"].measured_cp_world
        for config in self.configs:
            name = config.name
            state = replace(self.states[name], sequence=self.sequence, simulation_time=self.sequence / 200,
                            operating_state_event=False)
            for command in self.commands[name].drain():
                if command.channel == "state_command":
                    state = replace(state, operating_state=OperatingStateSnapshot(command.payload, True, False))
                elif command.channel == "servo_cp":
                    pose = resolve_cartesian_command(command.payload, config, previous_ecm, has_ecm=name == "PSM1")
                    state = replace(state, measured_cp_world=pose, setpoint_cp_world=pose)
                elif command.channel == "servo_jp":
                    state = replace(state, measured_js=JointState(("yaw",), command.payload, [0]))
            self.states[name] = state
        return with_publication_frames(self.states, self.configs)

    def take_camera_rate_hz(self):
        return 0.0

    def shutdown(self):
        pass


def fake_factory(start):
    if start.get("fail"):
        raise RuntimeError("intentional initialization failure")
    commands = {name: CommandMailboxes() for name in ("ECM", "PSM1")}
    return FakeRuntime(commands), commands


class FakeNode:
    def __init__(self):
        self.arm_interfaces = {name: SimpleNamespace(commands=CommandMailboxes(), snapshots=LatestSnapshot(),
                                                     _publish_warning=lambda text: None)
                               for name in ("ECM", "PSM1")}
        self.states = {}

    def install_initial_snapshots(self, snapshots):
        self.states = snapshots
        for name, state in snapshots.items():
            self.arm_interfaces[name].snapshots.set_initial(state)

    def accept_snapshots(self, snapshots):
        self.states = snapshots
        for name, state in snapshots.items():
            self.arm_interfaces[name].snapshots.set(state)

    def get_logger(self):
        return SimpleNamespace(info=lambda text: None)


@pytest.fixture
def worker(monkeypatch):
    original_popen = subprocess.Popen
    code = (f"import sys, socket; sys.path.insert(0, {str(Path(__file__).parent)!r}); "
            "from test_process import fake_factory; "
            "from dvrk_simulator_base.process_worker import run_worker; "
            "from dvrk_simulator_base.ipc import UnixSocketEndpoint; "
            "raise SystemExit(run_worker(UnixSocketEndpoint(socket.socket(fileno=int(sys.argv[1]))), fake_factory))")

    def launch(arguments, **kwargs):
        return original_popen([sys.executable, "-c", code, arguments[-1]], **kwargs)

    monkeypatch.setattr(ipc_runtime.subprocess, "Popen", launch)
    monkeypatch.delenv("DVRK_SIMULATOR_TEST_TIMEOUT", raising=False)


def pump_until(worker, node, predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        worker.poll(node)
        if time.monotonic() > deadline:
            raise AssertionError("IPC worker did not reach expected state")
        time.sleep(0.001)


def test_process_commands_frames_events_and_graceful_shutdown(worker):
    process = ipc_runtime.SimulationProcess(sys.executable, "dvrk_newton.simulation_worker", {})
    node = FakeNode()
    try:
        pump_until(process, node, lambda: process.ready)
        assert process.worker_pid != os.getpid()
        pose = Pose([2, 0, 0], np.eye(3))
        node.arm_interfaces["ECM"].commands.submit_servo("servo_cp", CartesianCommand(pose, "world"))
        pump_until(process, node, lambda: node.states["ECM"].measured_cp_world.position[0] == 2)
        node.arm_interfaces["PSM1"].commands.submit_servo("servo_cp", CartesianCommand(Pose([0.1, 0.2, 0.3], np.eye(3)), "ECM_view"))
        pump_until(process, node, lambda: node.states["PSM1"].measured_cp_world.position[0] > 2)
        state = node.states["PSM1"]
        np.testing.assert_allclose(state.measured_cp_world.position, [2.3, 0.1, 0.2])
        np.testing.assert_allclose(state.publication_frames.measured_cp.position, [0.1, 0.2, 0.3])
        assert node.states["ECM"].sequence == state.sequence
        for text in ("PAUSED", "ENABLED"):
            node.arm_interfaces["PSM1"].commands.submit_discrete("state_command", text)
            pump_until(process, node, lambda: node.states["PSM1"].operating_state.state == text)
        _, events = node.arm_interfaces["PSM1"].snapshots.get_with_events()
        assert [event.state for event in events] == ["PAUSED", "ENABLED"]
        node.arm_interfaces["PSM1"].commands.submit_servo("servo_jp", [0.4])
        pump_until(process, node, lambda: node.states["PSM1"].measured_js.position[0] == 0.4)
    finally:
        process.shutdown()
    assert process.process.returncode == 0


def test_initialization_failure_reaps_worker(worker):
    process = ipc_runtime.SimulationProcess(sys.executable, "dvrk_newton.simulation_worker", {"fail": True})
    try:
        with pytest.raises(SimulationProcessError, match="intentional initialization failure"):
            pump_until(process, FakeNode(), lambda: process.ready)
    finally:
        process.shutdown()
    assert process.process.returncode == 1


def test_unexpected_worker_exit_is_reported_and_reaped(worker):
    process = ipc_runtime.SimulationProcess(sys.executable, "dvrk_newton.simulation_worker", {})
    node = FakeNode()
    try:
        pump_until(process, node, lambda: process.ready)
        process.process.kill()
        process.process.wait(timeout=5)
        with pytest.raises((SimulationProcessError, OSError)):
            pump_until(process, node, lambda: process.stopped)
    finally:
        process.shutdown()
    assert process.process.returncode is not None



def test_shutdown_drains_a_worker_with_backpressured_snapshots(worker):
    process = ipc_runtime.SimulationProcess(sys.executable, "dvrk_newton.simulation_worker", {})
    try:
        pump_until(process, FakeNode(), lambda: process.ready)
        # Stop reading long enough to fill the socket, then request cleanup.
        time.sleep(1.0)
    finally:
        process.shutdown()
    assert process.process.returncode == 0
