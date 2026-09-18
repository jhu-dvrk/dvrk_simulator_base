import pytest

from dvrk_simulator_base import model_source
from dvrk_simulator_base.model_source import ModelSourceError


def test_locate_dvrk_model_uses_ament_index(monkeypatch, tmp_path):
    (tmp_path / "urdf" / "Virtual").mkdir(parents=True)
    monkeypatch.setattr(
        model_source, "get_package_share_directory", lambda package: str(tmp_path)
    )
    assert model_source.locate_dvrk_model() == tmp_path.resolve()


def test_locate_dvrk_model_validates_install(monkeypatch, tmp_path):
    monkeypatch.setattr(
        model_source, "get_package_share_directory", lambda package: str(tmp_path)
    )
    with pytest.raises(ModelSourceError, match="urdf/Virtual"):
        model_source.locate_dvrk_model()