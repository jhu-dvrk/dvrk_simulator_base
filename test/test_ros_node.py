"""Shared diagnostics count actual frontend work using measured intervals."""
from types import SimpleNamespace
import pytest
from builtin_interfaces.msg import Time
from dvrk_simulator_base import ros_node


def test_diagnostics_report_actual_scene_rates_and_worker_snapshot_age(monkeypatch):
    node = ros_node.SimulatorRosNode.__new__(ros_node.SimulatorRosNode)
    node._publishing_enabled = True
    node._snapshot_count, node._publication_count = 12, 8
    node._sample_at = 10.0
    node._runtime = SimpleNamespace(metrics={"simulation_hz": 120.0, "camera_hz": 30.0,
                                            "snapshot_age_ms": 4.5})
    published = []
    node._diagnostics = SimpleNamespace(publish=published.append)
    node.get_name = lambda: "test_simulator"
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    monkeypatch.setattr(ros_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(ros_node.time, "monotonic", lambda: 12.0)
    node._publish_diagnostics()
    values = {item.key: float(item.value) for item in published[0].status[0].values}
    assert values == {"simulation_hz": 120.0, "camera_hz": 30.0,
                      "snapshot_age_ms": 4.5, "snapshot_receive_hz": 6.0, "state_publish_hz": 4.0}
    assert node._snapshot_count == node._publication_count == 0


def test_scene_rejects_partial_or_incoherent_snapshots():
    node = ros_node.SimulatorRosNode.__new__(ros_node.SimulatorRosNode)
    node.arm_interfaces = {"ECM": None, "PSM1": None}
    state = SimpleNamespace(sequence=3, simulation_time=0.025)
    with pytest.raises(ValueError, match="configured scene"):
        node._check_scene({"PSM1": state})
    with pytest.raises(ValueError, match="different steps"):
        node._check_scene({"ECM": state, "PSM1": SimpleNamespace(sequence=4, simulation_time=0.03)})
