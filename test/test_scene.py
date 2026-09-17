from pathlib import Path

import numpy as np

from dvrk_simulator_base.scene import SceneResolver, load_scene_config


def test_scene_resolver_lists_available_scenes_and_search_paths(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    (second / "available.yaml").touch()
    resolver = SceneResolver((first, second), relative_root=tmp_path)

    assert resolver.resolve("available") == second / "available.yaml"
    assert resolver.available() == (second / "available.yaml",)

    try:
        resolver.resolve("missing")
    except FileNotFoundError as error:
        message = str(error)
    else:
        raise AssertionError("missing scene was unexpectedly resolved")
    assert "Searched scene paths:" in message
    assert str(first) in message
    assert str(second) in message
    assert "available.yaml" in message


def test_scene_loads_robots_assets_and_frame_overrides(tmp_path):
    arm_root = Path(__file__).parents[1] / "share" / "arms"
    scene = tmp_path / "scene.yaml"
    scene.write_text(
        """scene:
  name: two_arms
  frames:
    PSM1: {position: [0.1, 0.2, 0.3], orientation_xyzw: [0, 0, 0, 1]}
  robots:
    - {config: PSM1.yaml, instrument: '420006'}
    - {config: ECM.yaml, endoscope: Si_straight}
""",
        encoding="utf-8",
    )
    result = load_scene_config(scene, robot_config_root=arm_root)
    assert result.name == "two_arms"
    assert [robot.name for robot in result.robots] == ["PSM1", "ECM"]
    np.testing.assert_allclose(result.robots[0].base_position, [0.1, 0.2, 0.3])
    assert result.robots[0].instrument == "420006"
    assert result.robots[1].endoscope == "Si_straight"
