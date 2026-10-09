"""Common command line and startup for the ROS simulator frontends."""

import argparse
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args


def parse_command_line(args, description, *, device=False):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", type=Path, help="Backend runtime YAML")
    parser.add_argument("--scene", type=Path, action="append", help="scene YAML; may be repeated")
    parser.add_argument("--headless", choices=("true", "false"), help="override desktop viewer setting")
    if device:
        parser.add_argument("--device", help="Warp device, e.g. cuda:0 or cpu")
    return parser.parse_args(remove_ros_args(args))


def simulator_main(package, args, parse_args, load_config, resolve_scene, resolve_python, node_type, run):
    raw_args = list(sys.argv[1:] if args is None else args)
    options = parse_args(raw_args)
    path = options.config or Path(get_package_share_directory(package)) / "share" / f"{package.removeprefix('dvrk_')}.yaml"
    if options.config is None and not path.is_file():
        path = path.with_suffix(".yaml.example")
    try:
        config = load_config(path)
        selection = options.scene or config.scene
        if not selection:
            raise ValueError("a scene is required: specify --scene or scene in the runtime YAML")
        scenes = resolve_scene(path, selection)
        python = resolve_python(config.generated_root).path
    except (RuntimeError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    headless = config.headless if options.headless is None else options.headless == "true"
    start = dict(
        config=str(path.expanduser().resolve()),
        scene=[str(item) for item in scenes] if isinstance(scenes, (tuple, list)) else str(scenes),
        headless=headless or "DVRK_SIMULATOR_TEST_TIMEOUT" in os.environ,
    )
    if hasattr(options, "device"):
        start["device"] = options.device or config.device
    return run(
        lambda: node_type(scene_path=scenes, state_publish_rate_hz=config.state_publish_rate_hz,
                          command_queue_capacity=config.command_queue_capacity),
        python, f"{package}.simulation_worker", start, raw_args,
    )
