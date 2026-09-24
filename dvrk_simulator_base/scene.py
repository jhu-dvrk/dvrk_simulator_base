"""Backend-independent multi-arm scene configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import yaml

from dvrk_arm_description import RobotConfig, load_robot_config, with_base_pose


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

    def _resolve_single(self, selection: str | Path) -> Path:
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

    def resolve(self, selection: str | Path) -> Path:
        """Resolve a scene selection or report the available alternatives."""
        return self._resolve_single(selection)

    def resolve_all(self, selections: Sequence[str | Path]) -> tuple[Path, ...]:
        """Resolve multiple scene selections in order."""
        return tuple(self._resolve_single(item) for item in selections)


def _load_documents(
    source: Path,
    visited: set[Path],
    resolver: SceneResolver | None,
) -> list[tuple[Path, dict[str, Any]]]:
    resolved_source = source.resolve()
    if resolved_source in visited:
        raise ValueError(f"Circular scene include detected: {resolved_source}")
    visited.add(resolved_source)

    if not resolved_source.is_file():
        if resolver is not None:
            resolved_source = resolver.resolve(source)
            if resolved_source in visited:
                raise ValueError(f"Circular scene include detected: {resolved_source}")
            visited.add(resolved_source)
        else:
            raise FileNotFoundError(f"Scene configuration not found: {source}")

    with resolved_source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    scene = document.get("scene") if isinstance(document, dict) else None
    if not isinstance(scene, dict):
        raise ValueError(f"{resolved_source}: expected a top-level 'scene' mapping")

    includes = (
        scene.get("include")
        or scene.get("includes")
        or document.get("include")
        or document.get("includes")
    )
    if includes is not None:
        if isinstance(includes, (str, Path)):
            include_list = [includes]
        elif isinstance(includes, list):
            include_list = includes
        else:
            raise ValueError(
                f"{resolved_source}: 'include' must be a path or list of paths"
            )
    else:
        include_list = []

    documents: list[tuple[Path, dict[str, Any]]] = []
    for inc in include_list:
        inc_target = Path(str(inc)).expanduser()
        if inc_target.is_absolute() and inc_target.is_file():
            resolved_inc = inc_target.resolve()
        elif (resolved_source.parent / inc_target).is_file():
            resolved_inc = (resolved_source.parent / inc_target).resolve()
        elif (resolved_source.parent / f"{inc_target}.yaml").is_file():
            resolved_inc = (resolved_source.parent / f"{inc_target}.yaml").resolve()
        elif resolver is not None:
            resolved_inc = resolver.resolve(inc_target)
        else:
            raise FileNotFoundError(
                f"{resolved_source}: included scene file {inc!r} not found"
            )
        documents.extend(_load_documents(resolved_inc, visited, resolver))

    documents.append((resolved_source, scene))
    return documents


def load_scene_config(
    path: str | Path | Sequence[str | Path],
    *,
    robot_config_root: str | Path | None = None,
    resolver: SceneResolver | None = None,
) -> SceneConfig:
    """Load one or more scene files and resolve robots, frames, and objects."""
    if isinstance(path, (str, Path)):
        sources = [Path(path).expanduser()]
    elif isinstance(path, (list, tuple)):
        sources = [Path(p).expanduser() for p in path]
        if not sources:
            raise ValueError("at least one scene path must be provided")
    else:
        raise TypeError(f"expected path or sequence of paths, got {type(path)}")

    visited: set[Path] = set()
    all_documents: list[tuple[Path, dict[str, Any]]] = []
    for source in sources:
        all_documents.extend(_load_documents(source, visited, resolver))

    scene_names: list[str] = []
    merged_frames: dict[str, dict[str, Any]] = {}
    merged_camera_doc: dict[str, Any] = {}
    all_robot_entries: list[tuple[dict[str, Any], Path]] = []
    all_object_entries: list[tuple[dict[str, Any], Path]] = []

    for doc_source, scene in all_documents:
        name = scene.get("name")
        if name and str(name) not in scene_names:
            scene_names.append(str(name))

        frames = scene.get("frames", {})
        if frames:
            if not isinstance(frames, dict):
                raise ValueError(f"{doc_source}: scene.frames must be a mapping")
            merged_frames.update(frames)

        camera_doc = scene.get("camera")
        if camera_doc is not None:
            if not isinstance(camera_doc, dict):
                raise ValueError(f"{doc_source}: scene.camera must be a mapping")
            if camera_doc.get("mode", "off") != "off" or not merged_camera_doc:
                merged_camera_doc.update(camera_doc)

        robot_entries = scene.get("robots", [])
        if robot_entries:
            if not isinstance(robot_entries, list):
                raise ValueError(f"{doc_source}: scene.robots must be a list")
            for entry in robot_entries:
                all_robot_entries.append((entry, doc_source))

        object_entries = scene.get("objects", [])
        if object_entries:
            if not isinstance(object_entries, list):
                raise ValueError(f"{doc_source}: scene.objects must be a list")
            for entry in object_entries:
                all_object_entries.append((entry, doc_source))

    if not all_robot_entries:
        primary_source = all_documents[0][0]
        raise ValueError(f"{primary_source}: scene.robots must be a non-empty list")

    robots = []
    robot_names: set[str] = set()
    for entry, doc_source in all_robot_entries:
        if not isinstance(entry, dict) or "config" not in entry:
            raise ValueError(f"{doc_source}: each robot requires a config path")
        config_path = Path(str(entry["config"]))
        if not config_path.is_absolute():
            config_root = Path(robot_config_root or doc_source.parent).expanduser().resolve()
            config_path = config_root / config_path
        robot = load_robot_config(
            config_path,
            instrument=entry.get("instrument"),
            endoscope=entry.get("endoscope"),
        )
        frame = merged_frames.get(robot.name, {})
        if not isinstance(frame, dict):
            raise ValueError(f"{doc_source}: frame for {robot.name} must be a mapping")
        try:
            robot = with_base_pose(
                robot,
                position=frame.get("position"),
                orientation_xyzw=frame.get("orientation_xyzw"),
            )
        except (TypeError, ValueError) as error:
            raise ValueError(f"{doc_source}: invalid frame for {robot.name}: {error}") from error
        if robot.name in robot_names:
            raise ValueError(f"{doc_source}: duplicate robot {robot.name!r}")
        robot_names.add(robot.name)
        robots.append(robot)

    camera = SceneCamera(
        mode=str(merged_camera_doc.get("mode", "off")),
        owner=str(merged_camera_doc.get("owner", "ECM")),
        settings=dict(merged_camera_doc),
    )
    primary_source = all_documents[0][0]
    if camera.mode not in {"off", "mono", "stereo"}:
        raise ValueError(f"{primary_source}: camera.mode must be off, mono, or stereo")
    if camera.owner != "ECM":
        raise ValueError(f"{primary_source}: only ECM is supported as the camera owner")
    if camera.mode != "off" and camera.owner not in robot_names:
        raise ValueError(f"{primary_source}: camera owner {camera.owner!r} is not in the scene")

    objects = []
    object_names: set[str] = set()
    for entry, doc_source in all_object_entries:
        if not isinstance(entry, dict):
            raise ValueError(f"{doc_source}: each scene object must be a mapping")
        name = str(entry.get("name", ""))
        asset = str(entry.get("asset", ""))
        if not name or not asset:
            raise ValueError(f"{doc_source}: each scene object requires name and asset")
        if name in object_names or name in robot_names:
            raise ValueError(f"{doc_source}: duplicate scene object {name!r}")
        position = tuple(float(value) for value in entry.get("position", (0.0, 0.0, 0.0)))
        orientation = tuple(
            float(value) for value in entry.get("orientation_xyzw", (0.0, 0.0, 0.0, 1.0))
        )
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError(
                f"{doc_source}: scene object {name!r} requires 3D position and quaternion"
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

    scene_name = "+".join(scene_names) if scene_names else all_documents[0][0].stem
    return SceneConfig(scene_name, tuple(robots), camera, tuple(objects))
