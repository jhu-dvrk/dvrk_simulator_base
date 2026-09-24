"""Generate a portable rqt layout for simulator monitoring."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Iterable


def existing_ament_prefix_path() -> str | None:
    """Return AMENT_PREFIX_PATH without stale, deleted package prefixes."""
    value = os.environ.get("AMENT_PREFIX_PATH")
    if value is None:
        return None
    return os.pathsep.join(
        entry for entry in value.split(os.pathsep) if entry and Path(entry).is_dir()
    )


def write_monitor_perspective(
    path: str | Path,
    arms: Iterable[str],
    *,
    include_console: bool = False,
) -> Path:
    """Write a single-rqt perspective with a tabbed dVRK Arms dock."""
    arm_names = tuple(dict.fromkeys(str(name) for name in arms))
    plugins = {
        "rqt_dvrk/Arms": [1],
        "rqt_dvrk/Diagnostics": [1],
    }
    if include_console:
        plugins["rqt_dvrk/Console"] = [1]

    document = {
        "keys": {},
        "groups": {
            "pluginmanager": {
                "keys": {
                    "running-plugins": {"type": "repr", "repr": repr(plugins)},
                },
                "groups": {},
            },
            "mainwindow": {"keys": {}, "groups": {}},
        },
    }
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2), encoding="utf-8")
    return output
