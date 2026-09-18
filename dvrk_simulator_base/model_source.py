"""Locate model resources shared by simulator backends."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


class ModelSourceError(RuntimeError):
    """Raised when a shared model resource is unavailable or malformed."""


def locate_dvrk_model() -> Path:
    """Return the installed ``dvrk_model`` share directory.

    A missing package normally means the workspace containing ``dvrk_model``
    has not been built or sourced.
    """
    try:
        root = Path(get_package_share_directory("dvrk_model")).resolve()
    except PackageNotFoundError as error:
        raise ModelSourceError(
            "dvrk_model was not found in the ament index; build the workspace "
            "and source its install/setup.bash before starting a simulator"
        ) from error

    virtual = root / "urdf" / "Virtual"
    if not virtual.is_dir():
        raise ModelSourceError(
            f"the ament-index dvrk_model package has no urdf/Virtual directory: {root}"
        )
    return root