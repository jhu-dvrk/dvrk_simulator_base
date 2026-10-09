"""Shared multi-arm ROS frontend and metrics for IPC simulation backends."""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node

from .ros_interface import ArmRosInterface


class SimulatorRosNode(Node):
    def __init__(self, name, configs, *, state_publish_rate_hz=100.0, command_queue_capacity=32):
        if not configs:
            raise ValueError("a simulator requires a scene with configured arms")
        if state_publish_rate_hz <= 0:
            raise ValueError("state publication rate must be positive")
        super().__init__(name)
        self.configs = tuple(configs)
        self.arm_interfaces = {}
        ecm_config = next((config for config in configs if config.type == "ECM"), None)
        ecm = None
        if ecm_config is not None:
            ecm = ArmRosInterface(self, ecm_config, command_queue_capacity)
            self.arm_interfaces[ecm_config.name] = ecm
        for config in configs:
            if config is not ecm_config:
                self.arm_interfaces[config.name] = ArmRosInterface(
                    self, config, command_queue_capacity, ecm_interface=ecm,
                )
        self._runtime = None
        self._publishing_enabled = True
        self._snapshot_count = self._publication_count = 0
        self._sample_at = time.monotonic()
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._state_timer = self.create_timer(1.0 / state_publish_rate_hz, self._publish_latest)
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)

    @classmethod
    def from_scene_path(
        cls,
        name,
        scene_path,
        *,
        load_scene_fn,
        state_publish_rate_hz=100.0,
        command_queue_capacity=32,
    ):
        scene = load_scene_fn(scene_path)
        return cls(
            name,
            scene.robots,
            state_publish_rate_hz=state_publish_rate_hz,
            command_queue_capacity=command_queue_capacity,
        )

    def install_initial_snapshots(self, snapshots):
        self._check_scene(snapshots)
        for name, snapshot in snapshots.items():
            self.arm_interfaces[name].install_initial_snapshot(snapshot)

    def accept_snapshots(self, snapshots):
        self._check_scene(snapshots)
        for name, snapshot in snapshots.items():
            self.arm_interfaces[name].snapshots.set(snapshot)
        self._snapshot_count += 1

    def _check_scene(self, snapshots):
        if snapshots.keys() != self.arm_interfaces.keys():
            raise ValueError("simulation snapshot does not contain the configured scene")
        if len({(snapshot.sequence, snapshot.simulation_time) for snapshot in snapshots.values()}) != 1:
            raise ValueError("simulation snapshot contains different steps")

    def _publish_latest(self):
        if not self._publishing_enabled or not rclpy.ok():
            return
        try:
            if all(interface.snapshots.peek() is not None for interface in self.arm_interfaces.values()):
                for interface in self.arm_interfaces.values():
                    interface.publish_latest()
                self._publication_count += 1
        except Exception:
            if rclpy.ok():
                raise

    def _publish_diagnostics(self):
        if not self._publishing_enabled or not rclpy.ok():
            return
        now = time.monotonic()
        elapsed = max(now - self._sample_at, 1e-6)
        metrics = self._runtime.metrics if self._runtime else {
            "simulation_hz": 0.0, "camera_hz": 0.0, "snapshot_age_ms": 0.0,
        }
        metrics.update(state_publish_hz=self._publication_count / elapsed,
                       snapshot_receive_hz=self._snapshot_count / elapsed)
        self._publication_count = self._snapshot_count = 0
        self._sample_at = now
        status = DiagnosticStatus()
        status.name = f"{self.get_name()}/runtime"
        status.hardware_id = self.get_name()
        status.level = DiagnosticStatus.OK if metrics["simulation_hz"] > 0 else DiagnosticStatus.WARN
        status.message = "running" if metrics["simulation_hz"] > 0 else "waiting for simulation"
        status.values = [KeyValue(key=key, value=f"{value:.1f}") for key, value in metrics.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        try:
            self._diagnostics.publish(message)
        except Exception:
            if rclpy.ok():
                raise

    def stop_publishing(self):
        self._publishing_enabled = False
        self._state_timer.cancel()
        self._diagnostics_timer.cancel()


class SceneBasedSimulatorNode(SimulatorRosNode):
    """Convenience subclass that loads scene robots using a configuration loader."""

    def __init__(
        self,
        name,
        *,
        scene_path,
        load_scene_fn,
        state_publish_rate_hz=100.0,
        command_queue_capacity=32,
    ):
        scene = load_scene_fn(scene_path)
        super().__init__(
            name,
            scene.robots,
            state_publish_rate_hz=state_publish_rate_hz,
            command_queue_capacity=command_queue_capacity,
        )


def run_frontend(node_factory, python, worker_module, start, ros_args):
    """Own ROS and the backend lifecycle on one frontend thread."""
    import signal
    import sys
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from .process import SimulationProcess

    def stop(signum, frame):
        raise KeyboardInterrupt

    node = process = executor = None
    previous_sigterm = signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.init(args=ros_args)
        node = node_factory()
        process = SimulationProcess(python, worker_module, start)
        node._runtime = process
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not process.stopped:
            process.poll(node)
            if not process.stopped:
                executor.spin_once(timeout_sec=0.001)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, OSError, ValueError, BufferError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if node is not None:
                node.stop_publishing()
            if process is not None:
                process.shutdown()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)
    return 0
