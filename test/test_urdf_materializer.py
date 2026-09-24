import os
from pathlib import Path

from dvrk_simulator_base.urdf_materializer import (
    default_generated_root,
    materialize_virtual_psm,
    materialize_virtual_robot,
)


def test_generated_root_is_cache_directory():
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    assert default_generated_root() == cache_root / "dvrk_simulator_base"


def test_virtual_psm1_is_expanded_and_cached(tmp_path):
    first = materialize_virtual_psm(generated_root=tmp_path)
    second = materialize_virtual_psm(generated_root=tmp_path)

    assert first == second
    assert first.urdf_path.is_file()
    assert first.metadata_path.is_file()
    text = first.urdf_path.read_text(encoding="utf-8")
    assert '<robot name="PSM1">' in text
    assert 'joint name="yaw"' in text
    assert "package://dvrk_model/" not in text
    assert 'filename="/' in text


def test_virtual_ecm_is_expanded_and_cached(tmp_path):
    result = materialize_virtual_robot(
        "ECM", endoscope="Si_straight", generated_root=tmp_path
    )
    assert result.urdf_path.is_file()
    assert result.metadata_path.is_file()
    text = result.urdf_path.read_text(encoding="utf-8")
    assert '<robot name="ECM">' in text
    assert 'joint name="insertion"' in text
    assert "package://dvrk_model/" not in text
