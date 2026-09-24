from pathlib import Path
import sys

import pytest

from dvrk_simulator_base import python_runtime


def test_check_python_imports():
    current = Path(sys.executable)
    assert python_runtime.check_python_imports(current, "json")
    assert not python_runtime.check_python_imports(current, "nonexistent_fake_package_xyz123")


def test_explicit_interpreter_is_saved_and_reused(tmp_path, monkeypatch):
    interpreter = (tmp_path / "venv" / "bin" / "python").absolute()
    monkeypatch.setenv("DVRK_TEST_PYTHON", str(interpreter))

    explicit = python_runtime.resolve_simulator_python(
        simulator_name="TestSim",
        env_var="DVRK_TEST_PYTHON",
        generated_root=tmp_path,
        check_import_fn=lambda candidate: candidate == interpreter,
    )
    assert explicit.path == interpreter
    assert explicit.source == "DVRK_TEST_PYTHON"
    assert (tmp_path / "python-runtime.json").is_file()

    monkeypatch.delenv("DVRK_TEST_PYTHON")
    cached = python_runtime.resolve_simulator_python(
        simulator_name="TestSim",
        env_var="DVRK_TEST_PYTHON",
        generated_root=tmp_path,
        check_import_fn=lambda candidate: candidate == interpreter,
    )
    assert cached.path == interpreter
    assert cached.source == f"saved selection in {tmp_path / 'python-runtime.json'}"


def test_workspace_venv_discovery(tmp_path):
    venv_dir = tmp_path / "workspace" / ".venv-test" / "bin"
    venv_dir.mkdir(parents=True)
    fake_python = venv_dir / "python3"
    fake_python.touch()

    package_file = tmp_path / "workspace" / "src" / "pkg" / "module.py"
    package_file.parent.mkdir(parents=True)
    package_file.touch()

    cache_dir = tmp_path / "cache"

    result = python_runtime.resolve_simulator_python(
        simulator_name="TestSim",
        env_var="DVRK_TEST_PYTHON",
        generated_root=cache_dir,
        check_import_fn=lambda p: p == fake_python.absolute(),
        workspace_venv_names=(".venv-test",),
        source_file=package_file,
    )
    assert result.path == fake_python.absolute()
    assert "workspace venv" in result.source


def test_missing_python_raises_informative_error(tmp_path, monkeypatch):
    monkeypatch.delenv("DVRK_TEST_PYTHON", raising=False)
    with pytest.raises(RuntimeError, match="TestSim is unavailable"):
        python_runtime.resolve_simulator_python(
            simulator_name="TestSim",
            env_var="DVRK_TEST_PYTHON",
            generated_root=tmp_path,
            check_import_fn=lambda p: False,
            bootstrap_command="./bootstrap.sh",
        )
