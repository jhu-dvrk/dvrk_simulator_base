"""Configuration loading for backend-independent robot semantics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class JointConfig:
    name: str
    type: str
    lower: float
    upper: float
    velocity: float


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_robot_document(path: str | Path, _stack: tuple[Path, ...] = ()) -> dict[str, Any]:
    """Load a robot YAML document and resolve its relative ``include`` files."""
    source = Path(path).expanduser().resolve()
    if source in _stack:
        chain = " -> ".join(str(item) for item in (*_stack, source))
        raise ValueError(f"cyclic robot YAML include: {chain}")
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    includes = document.pop("include", [])
    if isinstance(includes, (str, Path)):
        includes = [includes]
    if not isinstance(includes, list):
        raise ValueError(f"{source}: include must be a path or list of paths")
    merged: dict[str, Any] = {}
    for include in includes:
        include_path = Path(include)
        if not include_path.is_absolute():
            include_path = source.parent / include_path
        merged = _deep_merge(merged, load_robot_document(include_path, (*_stack, source)))
    return _deep_merge(merged, document)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    type: str
    model: str
    instrument: str | None
    endoscope: str | None
    parent_frame: str
    base_frame: str
    rcm_frame: str
    tool_frame: str
    adaptor_frame: str
    base_position: np.ndarray
    base_orientation_xyzw: np.ndarray
    joints: tuple[JointConfig, ...]
    home_position: np.ndarray
    raw: dict[str, Any]


def load_robot_config(path: str | Path, base_position: Any | None = None,
                      base_orientation_xyzw: Any | None = None,
                      instrument: str | None = None,
                      endoscope: str | None = None) -> RobotConfig:
    """Load and validate one robot YAML configuration."""

    source = Path(path)
    document = load_robot_document(source)

    if not isinstance(document, dict) or not isinstance(document.get("robot"), dict):
        raise ValueError(f"{source}: expected a top-level 'robot' mapping")

    robot = document["robot"]
    asset = robot.setdefault("asset", {})
    if not isinstance(asset, dict):
        raise ValueError(f"{source}: robot.asset must be a mapping")
    if instrument is not None:
        asset["instrument"] = str(instrument)
    if endoscope is not None:
        asset["endoscope"] = str(endoscope)
    robot_type = str(robot["type"]).upper() if "type" in robot else ""
    if robot_type not in {"PSM", "ECM"}:
        raise ValueError(f"{source}: robot.type must be PSM or ECM")
    robot["type"] = robot_type
    required = ("name", "type", "model", "parent_frame", "base_frame",
                "rcm_frame", "tool_frame", "adaptor_frame", "joints",
                "home_position")
    missing = [key for key in required if key not in robot]
    if missing:
        raise ValueError(f"{source}: missing robot fields: {', '.join(missing)}")

    base_pose = robot.get("base_pose", {})
    position = np.asarray(base_pose.get("position", [0.0, 0.0, 0.0]), dtype=float)
    orientation = np.asarray(base_pose.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float)
    if position.shape != (3,):
        raise ValueError(f"{source}: base_pose.position must have three values")
    if orientation.shape != (4,):
        raise ValueError(f"{source}: base_pose.orientation_xyzw must have four values")
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
        raise ValueError(f"{source}: base_pose must contain only finite values")
    if np.linalg.norm(orientation) == 0.0:
        raise ValueError(f"{source}: base_pose.orientation_xyzw cannot be zero")

    joints = []
    for joint in robot["joints"]:
        try:
            item = JointConfig(
                name=str(joint["name"]),
                type=str(joint["type"]),
                lower=float(joint["lower"]),
                upper=float(joint["upper"]),
                velocity=float(joint["velocity"]),
            )
        except KeyError as error:
            raise ValueError(f"{source}: joint missing field {error.args[0]}") from error
        if item.type not in {"revolute", "prismatic"}:
            raise ValueError(f"{source}: unsupported joint type {item.type!r}")
        if (not np.all(np.isfinite([item.lower, item.upper, item.velocity]))
                or item.lower > item.upper or item.velocity <= 0.0):
            raise ValueError(f"{source}: invalid limits for joint {item.name!r}")
        joints.append(item)
    joint_names = [joint.name for joint in joints]
    if len(set(joint_names)) != len(joint_names):
        raise ValueError(f"{source}: joint names must be unique")

    home = np.asarray(robot["home_position"], dtype=float)
    if home.shape != (len(joints),):
        raise ValueError(f"{source}: home_position must match the joint count")
    if not np.all(np.isfinite(home)):
        raise ValueError(f"{source}: home_position must contain only finite values")
    for value, joint in zip(home, joints):
        if not joint.lower <= value <= joint.upper:
            raise ValueError(f"{source}: home position exceeds limits for {joint.name!r}")

    if base_position is not None:
        position = np.asarray(base_position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(
                f"{source}: overridden base position must have three finite values"
            )
    if base_orientation_xyzw is not None:
        orientation = np.asarray(base_orientation_xyzw, dtype=float)
        if (orientation.shape != (4,) or not np.all(np.isfinite(orientation))
                or np.linalg.norm(orientation) == 0.0):
            raise ValueError(f"{source}: overridden base orientation must be a non-zero quaternion")

    position = np.array(position, dtype=float, copy=True)
    orientation = np.array(orientation / np.linalg.norm(orientation), copy=True)
    home = np.array(home, dtype=float, copy=True)
    position.setflags(write=False)
    orientation.setflags(write=False)
    home.setflags(write=False)

    return RobotConfig(
        name=str(robot["name"]),
        type=robot_type,
        model=str(robot["model"]),
        instrument=(str(asset["instrument"])
                    if asset.get("instrument") is not None else None),
        endoscope=(str(asset["endoscope"])
                   if asset.get("endoscope") is not None else None),
        parent_frame=str(robot["parent_frame"]),
        base_frame=str(robot["base_frame"]),
        rcm_frame=str(robot["rcm_frame"]),
        tool_frame=str(robot["tool_frame"]),
        adaptor_frame=str(robot["adaptor_frame"]),
        base_position=position,
        base_orientation_xyzw=orientation,
        joints=tuple(joints),
        home_position=home,
        raw=document,
    )
