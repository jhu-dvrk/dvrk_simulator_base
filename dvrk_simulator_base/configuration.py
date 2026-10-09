"""Shared strict runtime configuration and installed scene resolution."""

from dataclasses import dataclass, fields
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from .scene import SceneResolver, load_scene_config


def check_keys(document, allowed, source, field="configuration"):
    if not isinstance(document, dict):
        raise ValueError(f"{source}: {field} must be a mapping")
    unknown = set(document) - set(allowed)
    if unknown:
        raise ValueError(f"{source}: unknown {field} options: {', '.join(sorted(map(str, unknown)))}")


def boolean(value, source, field):
    if not isinstance(value, bool):
        raise ValueError(f"{source}: {field} must be true or false")
    return value


def number(value, source, field, *, minimum=0.0, inclusive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{source}: {field} must be a number")
    value = float(value)
    if not math.isfinite(value) or (value < minimum if inclusive else value <= minimum):
        raise ValueError(f"{source}: invalid {field}; values and rates must be positive and finite")
    return value


def integer(value, source, field, *, minimum=1):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{source}: {field} must be an integer >= {minimum}")
    return value


@dataclass(frozen=True)
class RuntimeConfig:
    headless: bool = False
    simulation_rate_hz: float = 120.0
    state_publish_rate_hz: float = 100.0
    generated_root: Path | None = None
    command_queue_capacity: int = 32
    scene: str | None = None


@dataclass(frozen=True)
class PoseGraspConfig:
    max_grasps_per_object: int = 1
    close_threshold_rad: float = 0.04
    release_threshold_rad: float = 0.08
    break_distance_m: float = 0.005
    break_orientation_rad: float = 0.2617993877991494
    contact_region_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.003)
    contact_region_radius_m: float = 0.005


@dataclass(frozen=True)
class ConstraintGraspConfig(PoseGraspConfig):
    show_grasps: bool = True
    policy: str = "pose_error"
    arm_policies: dict[str, str] | None = None
    break_tension_force_n: float = 10.0
    break_shear_force_n: float = 10.0
    break_torque_nm: float = 0.25
    break_load_duration_s: float = 0.05
    max_force_n: float = 100.0
    constraint_erp: float = 0.8
    contact_region_radius_m: float = 0.008


def load_grasp(document, config_type, source):
    defaults = config_type()
    allowed = {field.name for field in fields(defaults)} - {"arm_policies"}
    if hasattr(defaults, "policy"):
        allowed.add("arms")
    check_keys(document, allowed, source, "grasp")
    values = {}
    for field in fields(defaults):
        name = field.name
        value = document.get(name, getattr(defaults, name))
        label = f"grasp.{name}"
        if name == "arm_policies":
            continue
        if name == "policy":
            if not isinstance(value, str) or value not in {"pose_error", "force_torque"}:
                raise ValueError(f"{source}: invalid {label} {value!r}")
        elif name == "show_grasps":
            value = boolean(value, source, label)
        elif name == "max_grasps_per_object":
            value = integer(value, source, label, minimum=0)
        elif name == "contact_region_offset_m":
            if not isinstance(value, (list, tuple)) or len(value) != 3:
                raise ValueError(f"{source}: {label} must contain three values")
            value = tuple(number(v, source, label, minimum=-math.inf) for v in value)
        else:
            value = number(value, source, label, inclusive=name in {"close_threshold_rad", "break_load_duration_s"})
        values[name] = value
    if values["release_threshold_rad"] <= values["close_threshold_rad"]:
        raise ValueError(f"{source}: grasp release threshold must exceed close threshold")
    if values["break_orientation_rad"] > math.pi or values.get("constraint_erp", 1.0) > 1.0:
        raise ValueError(f"{source}: invalid grasp tuning values")
    if hasattr(defaults, "policy"):
        arms = document.get("arms", {})
        check_keys(arms, arms.keys() if isinstance(arms, dict) else (), source, "grasp.arms")
        policies = {}
        for arm, settings in arms.items():
            check_keys(settings, {"policy"}, source, f"grasp.arms.{arm}")
            policy = settings.get("policy", values["policy"])
            if not isinstance(policy, str) or policy not in {"pose_error", "force_torque"}:
                raise ValueError(f"{source}: invalid grasp.arms.{arm}.policy {policy!r}")
            policies[str(arm)] = policy
        values["arm_policies"] = policies
    return config_type(**values)


def load_runtime_config(path, config_type, *, renderers=None, grasp_type=None):
    source = Path(path).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if document is None:
        document = {}
    defaults = config_type()
    check_keys(document, {field.name for field in fields(defaults)}, source)
    values = {}
    for field in fields(defaults):
        name = field.name
        value = document.get(name, getattr(defaults, name))
        if name == "headless":
            value = boolean(value, source, name)
        elif name == "command_queue_capacity":
            value = integer(value, source, name)
        elif name.endswith("_hz") or name == "rigid_gap_m":
            value = number(value, source, name)
        elif name == "generated_root":
            if value in (None, ""):
                value = None
            else:
                if not isinstance(value, (str, Path)):
                    raise ValueError(f"{source}: generated_root must be a path")
                value = Path(value).expanduser()
                value = (source.parent / value).resolve()
        elif name == "scene":
            if value in (None, ""):
                value = None
            elif not isinstance(value, str):
                raise ValueError(f"{source}: scene must be a path or filename")
        elif name == "renderer":
            if not isinstance(value, str) or value not in renderers:
                raise ValueError(f"{source}: unsupported renderer {value!r}")
        elif name == "device":
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{source}: device must be a non-empty string")
            value = value.strip()
        elif name == "grasp":
            value = load_grasp({} if value is None else value, grasp_type, source)
        values[name] = value
    return config_type(**values)


def scene_search_paths(package, config_path=None):
    share = Path(get_package_share_directory("dvrk_simulator_base"))
    backend = Path(get_package_share_directory(package))
    paths = [backend / "share/scenes", share / "share/scenes", share / "share/exercises"]
    if config_path is not None:
        paths.insert(0, Path(config_path).expanduser().resolve().parent / "scenes")
    return tuple(dict.fromkeys(path.resolve() for path in paths))


def resolve_scene_path(package, config_path, selection):
    config = Path(config_path).expanduser().resolve()
    resolver = SceneResolver(scene_search_paths(package, config), relative_root=config.parent)
    return resolver.resolve_all(selection) if isinstance(selection, (list, tuple)) else resolver.resolve(selection)


def load_installed_scene_config(package, path, *, search_paths=None):
    arms = Path(get_package_share_directory("dvrk_arm_description")) / "arms"
    resolver = SceneResolver(tuple(search_paths) if search_paths is not None else scene_search_paths(package))
    return load_scene_config(path, robot_config_root=arms, resolver=resolver)
