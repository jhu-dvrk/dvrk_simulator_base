import math

import pytest
import yaml

from dvrk_simulator_base.cart_frames import (
    DEFAULT_SPHERICAL_COORDINATES,
    SPHERE_RADIUS_M,
    rcm_frames,
    rcm_position,
    load_scene_coordinates,
    spherical_coordinates,
    yaml_frames,
    update_scene_file,
)


def test_default_ecm_is_at_the_front_of_the_rcm_sphere():
    position = rcm_position(*DEFAULT_SPHERICAL_COORDINATES["ECM"])
    assert position == pytest.approx((0.0, -0.17320508075688773, 0.10000000000000003))
    assert math.sqrt(sum(value * value for value in position)) == SPHERE_RADIUS_M


def test_frames_preserve_all_arms_and_produce_scene_yaml():
    frames = rcm_frames()
    assert tuple(frames) == ("PSM1", "PSM2", "PSM3", "ECM")
    assert frames["ECM"][1] == pytest.approx((0.5, 0.0, 0.0, 0.8660254037844386))
    document = yaml_frames()
    assert "frames:" in document
    assert "ECM: {position: [0.000000000, -0.173205081, 0.100000000]" in document


def test_update_scene_file_replaces_frames_and_preserves_scene_content(tmp_path):
    scene_file = tmp_path / "scene.yaml"
    scene_file.write_text("scene:\n  name: example\n  camera: {mode: off}\n", encoding="utf-8")
    update_scene_file(scene_file)
    document = yaml.safe_load(scene_file.read_text(encoding="utf-8"))
    assert document["scene"]["name"] == "example"
    assert document["scene"]["frames"]["ECM"]["position"] == [
        0.0,
        -0.17320508075688773,
        0.10000000000000003,
    ]
    assert load_scene_coordinates(scene_file)["ECM"] == pytest.approx((-90.0, 60.0))


def test_spherical_coordinates_rejects_center_position():
    with pytest.raises(ValueError, match="workspace center"):
        spherical_coordinates((0.0, 0.0, 0.0))
