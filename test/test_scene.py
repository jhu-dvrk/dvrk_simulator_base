from pathlib import Path

import numpy as np

from dvrk_simulator_base.scene import load_scene_config


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
