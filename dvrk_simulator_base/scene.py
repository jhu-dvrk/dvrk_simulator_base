"""Backend-independent multi-arm scene configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .config import RobotConfig, load_robot_config, with_base_pose


@dataclass(frozen=True)
class SceneCamera:
    """Backend-independent camera description retained from the scene YAML."""

    mode: str = "off"
    owner: str = "ECM"
    settings: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return dict(self.settings or {})


@dataclass(frozen=True)
class SceneObject:
    """Backend-neutral object declaration; backends resolve the asset URI."""

    name: str
    asset: str
    fixed: bool
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    settings: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return dict(self.settings or {})


@dataclass(frozen=True)
class SceneConfig:
    name: str
    robots: tuple[RobotConfig, ...]
    camera: SceneCamera = SceneCamera()
    objects: tuple[SceneObject, ...] = ()


@dataclass(frozen=True)
class SceneResolver:
    """Resolve scene files from backend-provided locations.

    The base package deliberately has no package-index dependency: each backend
    supplies its ordered scene directories and the directory used for explicit
    relative paths.
    """

    search_paths: tuple[Path, ...]
    relative_root: Path | None = None

    def __post_init__(self) -> None:
        unique_paths = []
        for path in self.search_paths:
            resolved = Path(path).expanduser().resolve()
            if resolved not in unique_paths:
                unique_paths.append(resolved)
        object.__setattr__(self, "search_paths", tuple(unique_paths))
        if self.relative_root is not None:
            object.__setattr__(
                self, "relative_root", Path(self.relative_root).expanduser().resolve()
            )

    def available(self) -> tuple[Path, ...]:
        """Return all YAML scenes in search order, without duplicates."""
        available = []
        for directory in self.search_paths:
            for path in sorted(directory.glob("*.yaml")):
                resolved = path.resolve()
                if resolved not in available:
                    available.append(resolved)
        return tuple(available)

    def resolve(self, selection: str | Path) -> Path:
        """Resolve a scene selection or report the available alternatives."""
        selected = Path(str(selection)).expanduser()
        if selected.is_absolute():
            candidates = (selected,)
        elif selected.parent == Path("."):
            candidates = tuple(directory / selected for directory in self.search_paths)
        elif self.relative_root is not None:
            candidates = (self.relative_root / selected,)
        else:
            candidates = (selected,)
        normalized = []
        for candidate in candidates:
            candidate = candidate.resolve()
            normalized.append(candidate)
            if not candidate.suffix:
                normalized.append(candidate.with_suffix(".yaml"))
        for candidate in normalized:
            if candidate.is_file():
                return candidate
        search_paths = "\n  ".join(str(path) for path in self.search_paths)
        available = "\n  ".join(path.name for path in self.available())
        raise FileNotFoundError(
            f"Scene configuration not found: {selection}\n"
            f"Searched scene paths:\n  {search_paths or '(none)'}\n"
            f"Available scenes:\n  {available or '(none)'}"
        )


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
        robot = load_robot_config(
            config_path,
            instrument=entry.get("instrument"),
            endoscope=entry.get("endoscope"),
        )
        frame = frames.get(robot.name, {})
        if not isinstance(frame, dict):
            raise ValueError(f"{source}: frame for {robot.name} must be a mapping")
        try:
            robot = with_base_pose(
                robot,
                position=frame.get("position"),
                orientation_xyzw=frame.get("orientation_xyzw"),
            )
        except (TypeError, ValueError) as error:
            raise ValueError(f"{source}: invalid frame for {robot.name}: {error}") from error
        if robot.name in names:
            raise ValueError(f"{source}: duplicate robot {robot.name!r}")
        names.add(robot.name)
        robots.append(robot)
    if camera.mode != "off" and camera.owner not in names:
        raise ValueError(f"{source}: camera owner {camera.owner!r} is not in the scene")

    object_entries = scene.get("objects", []) or []
    if not isinstance(object_entries, list):
        raise ValueError(f"{source}: scene.objects must be a list")
    objects = []
    object_names = set()
    for entry in object_entries:
        if not isinstance(entry, dict):
            raise ValueError(f"{source}: each scene object must be a mapping")
        name = str(entry.get("name", ""))
        asset = str(entry.get("asset", ""))
        if not name or not asset:
            raise ValueError(f"{source}: each scene object requires name and asset")
        if name in object_names or name in names:
            raise ValueError(f"{source}: duplicate scene object {name!r}")
        position = tuple(float(value) for value in entry.get("position", (0.0, 0.0, 0.0)))
        orientation = tuple(
            float(value) for value in entry.get("orientation_xyzw", (0.0, 0.0, 0.0, 1.0))
        )
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError(
                f"{source}: scene object {name!r} requires 3D position and quaternion"
            )
        object_names.add(name)
        objects.append(
            SceneObject(
                name=name,
                asset=asset,
                fixed=bool(entry.get("fixed", False)),
                position=position,
                orientation_xyzw=orientation,
                settings=dict(entry),
            )
        )
    return SceneConfig(str(scene.get("name", source.stem)), tuple(robots), camera, tuple(objects))
