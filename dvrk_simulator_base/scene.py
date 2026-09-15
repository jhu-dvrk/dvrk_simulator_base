"""Backend-independent multi-arm scene configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .config import RobotConfig, load_robot_config


@dataclass(frozen=True)
class SceneCamera:
    """Backend-independent camera description retained from the scene YAML."""

    mode: str = "off"
    owner: str = "ECM"
    settings: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return dict(self.settings or {})


@dataclass(frozen=True)
class SceneConfig:
    name: str
    robots: tuple[RobotConfig, ...]
    camera: SceneCamera = SceneCamera()


def load_scene_config(
    path: str | Path,
    *,
    robot_config_root: str | Path | None = None,
) -> SceneConfig:
    """Load a scene and resolve its robot configurations and base poses."""
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    scene = document.get("scene") if isinstance(document, dict) else None
    if not isinstance(scene, dict):
        raise ValueError(f"{source}: expected a top-level 'scene' mapping")
    entries = scene.get("robots")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{source}: scene.robots must be a non-empty list")
    frames = scene.get("frames", {})
    if not isinstance(frames, dict):
        raise ValueError(f"{source}: scene.frames must be a mapping")
    config_root = Path(robot_config_root or source.parent).expanduser().resolve()

    camera_document = scene.get("camera", {}) or {}
    if not isinstance(camera_document, dict):
        raise ValueError(f"{source}: scene.camera must be a mapping")
    camera = SceneCamera(
        mode=str(camera_document.get("mode", "off")),
        owner=str(camera_document.get("owner", "ECM")),
        settings=dict(camera_document),
    )
    if camera.mode not in {"off", "mono", "stereo"}:
        raise ValueError(f"{source}: camera.mode must be off, mono, or stereo")
    if camera.owner != "ECM":
        raise ValueError(f"{source}: only ECM is supported as the camera owner")

    robots = []
    names = set()
    for entry in entries:
        if not isinstance(entry, dict) or "config" not in entry:
            raise ValueError(f"{source}: each robot requires a config path")
        config_path = Path(str(entry["config"]))
        if not config_path.is_absolute():
            config_path = config_root / config_path
        provisional = load_robot_config(
            config_path,
            instrument=entry.get("instrument"),
            endoscope=entry.get("endoscope"),
        )
        frame = frames.get(provisional.name, {})
        if not isinstance(frame, dict):
            raise ValueError(f"{source}: frame for {provisional.name} must be a mapping")
        robot = load_robot_config(
            config_path,
            base_position=frame.get("position"),
            base_orientation_xyzw=frame.get("orientation_xyzw"),
            instrument=entry.get("instrument"),
            endoscope=entry.get("endoscope"),
        )
        if robot.name in names:
            raise ValueError(f"{source}: duplicate robot {robot.name!r}")
        names.add(robot.name)
        robots.append(robot)
    if camera.mode != "off" and camera.owner not in names:
        raise ValueError(f"{source}: camera owner {camera.owner!r} is not in the scene")
    return SceneConfig(str(scene.get("name", source.stem)), tuple(robots), camera)
