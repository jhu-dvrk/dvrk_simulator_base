from pathlib import Path

import numpy as np
import pytest

from dvrk.config import load_robot_config, load_robot_document


ROOT = Path(__file__).parents[1]


def test_shipped_robot_configs_load_without_urdf_parsing():
    expected = {"PSM1": 6, "PSM2": 6, "PSM3": 6, "ECM": 4}
    for name, joint_count in expected.items():
        config = load_robot_config(ROOT / "share" / "arms" / f"{name}.yaml")
        assert config.name == name
        assert len(config.joints) == joint_count
        assert config.model.endswith(".urdf.xacro")
        assert not config.home_position.flags.writeable


def test_asset_selection_can_be_overridden_without_interpreting_model():
    config = load_robot_config(
        ROOT / "share" / "arms" / "PSM1.yaml", instrument="CUSTOM_TOOL"
    )
    assert config.instrument == "CUSTOM_TOOL"
    assert config.endoscope is None


def test_relative_includes_deep_merge_mappings_and_replace_lists(tmp_path):
    parent = tmp_path / "parent.yaml"
    child = tmp_path / "child.yaml"
    parent.write_text(
        "robot:\n  nested: {left: 1, right: 2}\n  values: [1, 2]\n",
        encoding="utf-8",
    )
    child.write_text(
        "include: parent.yaml\nrobot:\n  nested: {right: 3}\n  values: [4]\n",
        encoding="utf-8",
    )

    document = load_robot_document(child)
    assert document["robot"]["nested"] == {"left": 1, "right": 3}
    assert document["robot"]["values"] == [4]


def test_include_cycle_is_rejected(tmp_path):
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    first.write_text("include: second.yaml\n", encoding="utf-8")
    second.write_text("include: first.yaml\n", encoding="utf-8")

    with pytest.raises(ValueError, match="cyclic"):
        load_robot_document(first)


def test_non_finite_configuration_is_rejected(tmp_path):
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(
        """robot:
  name: PSM1
  type: PSM
  model: Virtual/PSM1.urdf.xacro
  parent_frame: world
  base_frame: PSM1_base
  rcm_frame: PSM1_RCM
  tool_frame: PSM1_tool
  adaptor_frame: PSM1_adaptor
  joints:
    - {name: yaw, type: revolute, lower: -1.0, upper: 1.0, velocity: 1.0}
  home_position: [.nan]
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="finite"):
        load_robot_config(config_path)


def test_base_quaternion_is_normalized_and_read_only():
    config = load_robot_config(
        ROOT / "share" / "arms" / "ECM.yaml",
        base_orientation_xyzw=[0.0, 0.0, 0.0, 2.0],
    )
    np.testing.assert_allclose(config.base_orientation_xyzw, [0.0, 0.0, 0.0, 1.0])
    assert not config.base_orientation_xyzw.flags.writeable
