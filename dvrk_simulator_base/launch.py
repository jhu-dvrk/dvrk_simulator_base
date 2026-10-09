"""ROS launch construction shared by simulator and console profiles."""

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .rqt_perspective import existing_ament_prefix_path, write_monitor_perspective


def default_config(package):
    share = Path(get_package_share_directory(package))
    path = share / "share" / f"{package.removeprefix('dvrk_')}.yaml"
    return path if path.is_file() else path.with_suffix(".yaml.example")


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _boolean(context, name):
    value = _value(context, name).lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


@dataclass(frozen=True)
class SystemProfile:
    config: Path
    cart_scene: str | Path
    system_config: Path
    cwd: Path
    display_config: Path | None = None
    control_panel: bool = False
    auto_start: bool = False
    preview: bool = False


def start_session(context, package, profile=None):
    configuration = import_module(f"{package}.configuration")
    config_path = Path(_value(context, "config")).expanduser().resolve()
    config = configuration.load_simulator_config(config_path)
    selection = _value(context, "scene") or config.scene
    scenes = ([str(profile.cart_scene)] if profile else [])
    if selection:
        scenes.append(selection)
    if not scenes:
        raise ValueError("a scene is required")
    resolved = configuration.resolve_scene_path(config_path, scenes)
    cmd = [sys.executable, str(Path(get_package_share_directory(package)) / "scripts/simulator.py"),
           "--config", str(config_path)]
    for scene in resolved:
        cmd.extend(["--scene", str(scene)])
    if _value(context, "headless"):
        cmd.extend(["--headless", "true" if _boolean(context, "headless") else "false"])
    if package == "dvrk_newton" and _value(context, "device"):
        cmd.extend(["--device", _value(context, "device")])
    simulator = ExecuteProcess(cmd=cmd, output="screen")
    processes = [simulator]
    supervised = [simulator]
    if profile:
        system = Node(package="dvrk_robot", executable="dvrk_system", output="screen",
                      cwd=str(profile.cwd), arguments=["--json-config", str(profile.system_config)])
        processes.append(system)
        supervised.append(system)
        if profile.display_config:
            display = Node(package="dvrk_console", executable="stereo_display", output="screen",
                           arguments=["-c", str(profile.display_config)])
            processes.append(display)
            supervised.append(display)
        if profile.control_panel:
            processes.append(Node(package="dvrk_console", executable="control_panel", output="screen"))
        if profile.auto_start:
            processes.append(Node(package="dvrk_simulator_base", executable="start_dvrk_system",
                                  output="screen", arguments=["--console", _value(context, "console")]))
        if profile.preview and _boolean(context, "preview"):
            processes.append(TimerAction(period=2.0, actions=[ExecuteProcess(cmd=[
                "gst-launch-1.0", "unixfdsrc", "socket-path=dvrk:simulator:stereo_source",
                "socket-type=abstract", "do-timestamp=true", "!", "queue",
                "leaky=downstream", "max-size-buffers=1", "!", "videoconvert", "!",
                "autovideosink", "sync=false"], output="screen")]))
    if _boolean(context, "rqt"):
        scene = configuration.load_installed_scene_config(resolved)
        arms = [robot.name for robot in scene.robots]
        runtime = import_module(f"{package}.python_runtime")
        root = config.generated_root or runtime.default_generated_root()
        perspective = write_monitor_perspective(
            root / "rqt" / "monitor.perspective", arms,
            include_console=bool(profile) or _boolean(context, "rqt_console"))
        env = {"DVRK_RQT_ARMS": ",".join(arms), "DVRK_RQT_CONSOLE": _value(context, "console")}
        if prefix := existing_ament_prefix_path():
            env["AMENT_PREFIX_PATH"] = prefix
        processes.append(ExecuteProcess(cmd=[sys.executable, "-m", "dvrk_simulator_base.rqt",
                                            "--perspective-file", str(perspective)],
                                        additional_env=env, output="screen"))
    # Register supervision before starting processes, including fast failures.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason="simulator session process exited"))],
    )) for process in supervised]
    return handlers + processes


def simulator_launch(package, *, profile=None, exercise_argument="scene"):
    config = profile.config if profile else default_config(package)
    arguments = [
        DeclareLaunchArgument("config", default_value=str(config), description="Backend runtime YAML"),
        DeclareLaunchArgument(exercise_argument, default_value="tray_cubes.yaml" if profile else "",
                             description="Scene/exercise YAML path or installed filename"),
        DeclareLaunchArgument("headless", default_value="true" if profile else "",
                             description="Desktop viewer override; empty uses runtime YAML"),
        DeclareLaunchArgument("rqt", default_value="false", description="Start the dVRK monitor"),
        DeclareLaunchArgument("rqt_console", default_value="false", description="Include console monitoring"),
        DeclareLaunchArgument("console", default_value="console", description="Console namespace for clients"),
    ]
    if exercise_argument != "scene":
        arguments.append(DeclareLaunchArgument("scene", default_value=LaunchConfiguration(exercise_argument)))
    if package == "dvrk_newton":
        arguments.append(DeclareLaunchArgument("device", default_value="", description="Warp device override"))
    if profile and profile.preview:
        arguments.append(DeclareLaunchArgument("preview", default_value="true", description="Preview stereo video"))
    return LaunchDescription(arguments + [OpaqueFunction(function=start_session, args=[package, profile])])


def jhu_launch(platform, backend):
    directory = Path(get_package_share_directory("dvrk_config_jhu")).parent / platform
    profile = directory / backend
    label = {"newton": "Newton", "pybullet": "PyBullet", "isaac_sim": "IsaacSim"}[backend]
    return simulator_launch(f"dvrk_{backend}", profile=SystemProfile(
        config=profile / f"{backend}_patient_cart.yaml",
        cart_scene=profile / "ECM_PSM1_PSM2_PSM3.yaml",
        system_config=profile / f"system-MTMR-MTML-{label}-Teleop.json", cwd=directory,
        display_config=profile / "stereo_display_simulator.json", control_panel=True,
    ), exercise_argument="exercise")


def open_xr_launch(backend):
    package = f"dvrk_{backend}"
    directory = Path(get_package_share_directory(package)) / "share/open-xr"
    return simulator_launch(package, profile=SystemProfile(
        config=directory / f"{backend}.yaml",
        cart_scene="ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml" if backend == "isaac_sim" else "ECM_PSM1_PSM2_PSM3.yaml",
        system_config=directory / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json", cwd=directory,
        display_config=None if backend == "isaac_sim" else directory / "dvrk-console-overlay.json",
        auto_start=True,
    ))


def scene_test_launch(package):
    """Run a headless scene for a bounded interval after worker startup."""
    share = Path(get_package_share_directory(package))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(default_config(package))),
        DeclareLaunchArgument("scene", description="Scene YAML path or installed filename"),
        DeclareLaunchArgument("timeout", default_value="1.0", description="Test duration after initialization"),
        SetEnvironmentVariable("DVRK_SIMULATOR_TEST_TIMEOUT", LaunchConfiguration("timeout")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/simulator.launch.py")),
            launch_arguments={"config": LaunchConfiguration("config"),
                              "scene": LaunchConfiguration("scene"), "headless": "true"}.items(),
        ),
    ])
