"""Launch overrides must preserve runtime configuration and ROS Python."""
from pathlib import Path
import sys
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.utilities import perform_substitutions
import pytest

from dvrk_simulator_base.launch import start_session
from dvrk_simulator_base import launch as launch_module
from dvrk_simulator_base.configuration import RuntimeConfig, load_runtime_config
from dvrk_simulator_base.scene import SceneResolver


@pytest.mark.parametrize('configured,override', [(True, ''), (False, ''), (True, 'false'), (False, 'true')])
def test_simulator_uses_ros_python_and_only_explicit_headless_override(tmp_path, configured, override, monkeypatch):
    configuration = SimpleNamespace(
        load_simulator_config=lambda path: load_runtime_config(path, RuntimeConfig),
        resolve_scene_path=lambda path, selected: SceneResolver(()).resolve_all(selected),
    )
    monkeypatch.setattr(launch_module, 'import_module', lambda package: configuration)
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda package: str(tmp_path))
    config = tmp_path / 'pybullet.yaml'
    config.write_text(f'headless: {str(configured).lower()}\n')
    scene = tmp_path / 'scene.yaml'
    scene.write_text('scene: {robots: [{config: ECM.yaml}]}\n')
    context = LaunchContext()
    context.launch_configurations.update(config=str(config), scene=str(scene),
                                         headless=override, rqt='false')
    actions = start_session(context, 'dvrk_pybullet')
    assert isinstance(actions[0], RegisterEventHandler)
    process = next(action for action in actions if isinstance(action, ExecuteProcess))
    command = [perform_substitutions(context, item) for item in process.cmd]
    assert command[0] == sys.executable
    assert Path(command[1]).name == 'simulator.py'
    assert command[command.index('--scene') + 1] == str(scene)
    if override:
        assert command[command.index('--headless') + 1] == override
    else:
        assert '--headless' not in command
