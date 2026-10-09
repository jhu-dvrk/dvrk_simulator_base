"""Strict shared runtime fields prevent silent no-op settings."""
from dataclasses import dataclass, field

import pytest

from dvrk_simulator_base.configuration import PoseGraspConfig, RuntimeConfig, load_runtime_config


@dataclass(frozen=True)
class Config(RuntimeConfig):
    grasp: PoseGraspConfig = field(default_factory=PoseGraspConfig)


@pytest.mark.parametrize('document', [
    'headless: "false"', 'simulation_rate_hz: .nan', 'state_publish_rate_hz: 0',
    'command_queue_capacity: true', 'command_queue_capacity: 1.5', 'generated_root: []',
    'unused_option: true', 'grasp: false', 'grasp: []', 'grasp: {policy: force_torque}',
])
def test_invalid_or_ineffective_options_are_rejected(tmp_path, document):
    path = tmp_path / 'runtime.yaml'
    path.write_text(document)
    with pytest.raises(ValueError):
        load_runtime_config(path, Config, grasp_type=PoseGraspConfig)


def test_defaults_and_relative_cache_directory(tmp_path):
    path = tmp_path / 'runtime.yaml'
    path.write_text('generated_root: generated\ngrasp: {max_grasps_per_object: 0}\n')
    config = load_runtime_config(path, Config, grasp_type=PoseGraspConfig)
    assert config.generated_root == tmp_path / 'generated'
    assert config.simulation_rate_hz == 120.0
    assert config.grasp.max_grasps_per_object == 0
