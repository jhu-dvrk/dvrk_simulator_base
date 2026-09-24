"""Runtime selection and persistence for simulator Python interpreters."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Callable, Sequence, TypeVar


CACHE_FILE_NAME = "python-runtime.json"


@dataclass(frozen=True)
class SimulatorPython:
    path: Path
    source: str


T = TypeVar("T", bound=SimulatorPython)


def check_python_imports(python: Path, required_modules: Sequence[str] | str) -> bool:
    """Return True if the python binary can import all required modules."""
    if not python.is_file() or not os.access(python, os.X_OK):
        return False
    if isinstance(required_modules, str):
        modules = [required_modules]
    else:
        modules = list(required_modules)
    script = f"import {', '.join(modules)}"
    try:
        result = subprocess.run(
            [str(python), "-c", script],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except OSError:
        return False


def read_cached_python(cache_path: Path) -> Path | None:
    """Read a saved python interpreter path from a JSON cache file."""
    try:
        document = json.loads(cache_path.read_text(encoding="utf-8"))
        value = document.get("python")
        if not isinstance(value, str) or not value:
            return None
        return Path(value).expanduser().absolute()
    except (OSError, ValueError, TypeError):
        return None


def save_cached_python(cache_path: Path, python: Path) -> None:
    """Persist an interpreter choice without touching package source."""
    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_suffix(f".tmp-{os.getpid()}")
        temporary.write_text(
            json.dumps({"python": str(python)}, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(cache_path)
    except OSError:
        return


def find_workspace_venv(
    source_file: str | Path | None,
    venv_names: Sequence[str],
    check_fn: Callable[[Path], bool],
) -> tuple[Path, str] | None:
    """Search ancestor directories for a valid virtualenv binary."""
    origin = Path(source_file or __file__).resolve()
    for parent in origin.parents:
        for venv_name in venv_names:
            candidate = parent / venv_name / "bin" / "python3"
            if not candidate.is_file():
                candidate = parent / venv_name / "bin" / "python"
            if candidate.is_file():
                resolved = candidate.absolute()
                if check_fn(resolved):
                    return resolved, f"workspace venv at {resolved}"
    return None


def resolve_simulator_python(
    *,
    simulator_name: str,
    env_var: str,
    generated_root: str | Path | None = None,
    default_generated_root: Path | None = None,
    check_import_fn: Callable[[Path], bool] | None = None,
    required_modules: Sequence[str] | str | None = None,
    workspace_venv_names: Sequence[str] = (".venv",),
    bootstrap_command: str | None = None,
    result_factory: type[T] = SimulatorPython,
    source_file: str | Path | None = None,
) -> T:
    """Select a Python interpreter following the standard dVRK precedence rules:

    1. Explicit environment variable override.
    2. Cached interpreter from a previous run.
    3. Workspace virtual environment (.venv / .venv-<simulator>).
    4. Current running Python interpreter.
    """
    if check_import_fn is not None:
        check_fn = check_import_fn
    elif required_modules is not None:
        check_fn = lambda p: check_python_imports(p, required_modules)
    else:
        raise ValueError("Either check_import_fn or required_modules must be provided")

    root = Path(
        generated_root
        or default_generated_root
        or (Path.home() / ".cache" / f"dvrk_{simulator_name.lower()}")
    ).expanduser().resolve()
    cache_path = root / CACHE_FILE_NAME

    # 1. Environment variable override
    configured = os.environ.get(env_var, "").strip()
    if configured:
        candidate = Path(configured).expanduser().absolute()
        if not check_fn(candidate):
            raise RuntimeError(
                f"{env_var} is '{candidate}', but it cannot import required modules for {simulator_name}"
            )
        save_cached_python(cache_path, candidate)
        return result_factory(candidate, env_var)

    # 2. Saved cache
    cached = read_cached_python(cache_path)
    if cached is not None and check_fn(cached):
        return result_factory(cached, f"saved selection in {cache_path}")

    # 3. Workspace venv
    venv_result = find_workspace_venv(source_file, workspace_venv_names, check_fn)
    if venv_result is not None:
        venv_path, venv_desc = venv_result
        save_cached_python(cache_path, venv_path)
        return result_factory(venv_path, venv_desc)

    # 4. Current Python
    current = Path(sys.executable).absolute()
    if check_fn(current):
        save_cached_python(cache_path, current)
        return result_factory(current, "current Python")

    # Failure message
    hint = f"Run the bootstrap script:\n  {bootstrap_command}\n" if bootstrap_command else ""
    raise RuntimeError(
        f"{simulator_name} is unavailable in the current Python environment and no valid "
        f"saved interpreter was found in {cache_path}.\n"
        f"{hint}"
        f"Or set {env_var} to a Python interpreter with the required dependencies."
    )
