from pathlib import Path

import numpy as np

from dvrk_simulator_base.scene import SceneResolver, load_scene_config


ARM_ROOT = Path(__file__).parents[2] / "dvrk_arm_description" / "arms"


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
    result = load_scene_config(scene, robot_config_root=ARM_ROOT)
    assert result.name == "two_arms"
    assert [robot.name for robot in result.robots] == ["PSM1", "ECM"]
    np.testing.assert_allclose(result.robots[0].base_position, [0.1, 0.2, 0.3])
    assert result.robots[0].instrument == "420006"
    assert result.robots[1].endoscope == "Si_straight"


def test_scene_resolver_resolve_all(tmp_path):
    first = tmp_path / "scenes"
    first.mkdir()
    (first / "scene_a.yaml").touch()
    (first / "scene_b.yaml").touch()
    resolver = SceneResolver((first,))

    resolved = resolver.resolve_all(["scene_a", "scene_b"])
    assert resolved == (first / "scene_a.yaml", first / "scene_b.yaml")


def test_scene_loads_multiple_scene_files(tmp_path):
    scene_robots = tmp_path / "robots.yaml"
    scene_robots.write_text(
        """scene:
  name: cart
  robots:
    - {config: PSM1.yaml, instrument: '420006'}
""",
        encoding="utf-8",
    )
    scene_exercise = tmp_path / "exercise.yaml"
    scene_exercise.write_text(
        """scene:
  name: task
  objects:
    - name: table
      asset: package://dvrk_simulator_base/share/assets/table/table.urdf
      fixed: true
      position: [0.0, 0.0, 0.0]
      orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
""",
        encoding="utf-8",
    )

    result = load_scene_config(
        [scene_robots, scene_exercise], robot_config_root=ARM_ROOT
    )
    assert result.name == "cart+task"
    assert [robot.name for robot in result.robots] == ["PSM1"]
    assert len(result.objects) == 1
    assert result.objects[0].name == "table"


def test_scene_loads_included_scene_files(tmp_path):
    exercise = tmp_path / "exercise.yaml"
    exercise.write_text(
        """scene:
  name: exercise
  objects:
    - name: table
      asset: package://dvrk_simulator_base/share/assets/table/table.urdf
      fixed: true
      position: [0.0, 0.0, 0.0]
      orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
""",
        encoding="utf-8",
    )
    main_scene = tmp_path / "main.yaml"
    main_scene.write_text(
        """scene:
  name: main_scene
  include:
    - exercise.yaml
  robots:
    - {config: PSM1.yaml, instrument: '420006'}
""",
        encoding="utf-8",
    )

    result = load_scene_config(main_scene, robot_config_root=ARM_ROOT)
    assert "main_scene" in result.name
    assert [robot.name for robot in result.robots] == ["PSM1"]
    assert len(result.objects) == 1
    assert result.objects[0].name == "table"


def test_scene_detects_circular_includes(tmp_path):
    scene_a = tmp_path / "a.yaml"
    scene_b = tmp_path / "b.yaml"
    scene_a.write_text(
        """scene:
  name: a
  include: [b.yaml]
  robots: [{config: PSM1.yaml}]
""",
        encoding="utf-8",
    )
    scene_b.write_text(
        """scene:
  name: b
  include: [a.yaml]
""",
        encoding="utf-8",
    )

    try:
        load_scene_config(scene_a)
    except ValueError as error:
        assert "Circular scene include detected" in str(error)
    else:
        raise AssertionError("expected circular include error was not raised")
