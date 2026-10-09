# dvrk_simulator_base

Shared, simulator-independent contracts and CRTK behavior for dVRK simulation
backends. This package must not import Isaac Sim, Newton, Warp, or PyBullet.

The package provides immutable state types, rotation conversions, Cartesian
frame utilities, strict runtime configuration, scene composition, and common
CRTK command handling. All three simulators use `ArmController` for operating
state and independent arm/jaw trajectories; backends supply their own IK and
engine integration. Newton and Isaac share the URDF FK/Jacobian evaluator.

The shared [CRTK ROS contract](docs/ros_crtk_contract.md) defines the common
topic graph, QoS, command queueing, and Cartesian-frame behavior for backends.

Simulator engine model creation and physics integration remain backend-owned.
The base package provides shared, engine-independent URDF helpers—such as
`UrdfChain` for root-to-tool forward kinematics and analytic spatial Jacobians,
and `urdf_materializer` for simulation meshes—while describing semantic joint and
frame names in configuration and requiring contract-based mappings rather than
tying simulation logic to a specific physics engine.

Run the simulator-free tests with:

```shell
python3 -m pytest -q
```

## Patient-cart RCM frames

The base package owns the backend-neutral spherical RCM layout generator.
Print the default `frames:` YAML snippet with:

```shell
ros2 run dvrk_simulator_base generate_cart_frames
```

To update the `scene.frames` mapping of an existing scene in place:

```shell
ros2 run dvrk_simulator_base generate_cart_frames -s scene.yaml
```

For an interactive PyQt6 editor with top and side layout previews, adjustable
azimuth/polar coordinates for all RCMs, and a copyable YAML result:

```shell
ros2 run dvrk_simulator_base cart_frame_editor
```

PyQt6 is intentionally optional: it is required only for the editor, not for
using simulator backends or generating YAML from the command line.

## Unix socket process boundary

`ipc.UnixSocketEndpoint` provides versioned JSON framing over a private Unix
stream socket. Endpoints have one owner thread, bounded nonblocking writes,
ordered reliable messages, and replaceable unsent snapshots. Partially sent
frames are never replaced. Serialized values include commands, complete arm
snapshots, operating-state events, and simulation-owned publication frames;
ROS messages, callbacks, and backend objects cannot be serialized.

`CartesianCommand` carries a validated pose and its original `frame_id`.
`resolve_cartesian_command` runs in the simulator using authoritative ECM
state. `with_publication_frames` derives all Cartesian publication values
from the same completed scene. `ArmRosInterface` publishes these values and
passes Cartesian commands through without converting their reference frames.

`SimulatorRosNode`, `SimulationProcess`, and `process_worker.run_worker` own
shared ROS publication, session startup, one-batch command acknowledgement,
ordered event delivery, step pacing, diagnostics, and supervised shutdown.
Newton, PyBullet, and Isaac provide scene initialization and engine-specific
runtime steps. The simulation interpreter is selected independently of ROS Python.
The worker and transport have no ROS or GPU dependencies. All three frontends run under ROS Python and start
a separate worker with the selected simulation interpreter.

All IPC backends publish the same five metrics on `/diagnostics`:

- `simulation_hz`: completed world steps per wall-clock second in the worker.
- `camera_hz`: completed frames pushed to the camera video sink per second.
- `state_publish_hz`: actual complete-scene ROS publication cycles per second.
- `snapshot_receive_hz`: complete-scene snapshots accepted by ROS per second.
- `snapshot_age_ms`: elapsed time since the last accepted snapshot was captured
  in the simulation process, including time spent in IPC queues and transit.

Rates use measured sampling intervals. Camera rate is zero when disabled;
state publication is zero until initialization finishes. The frontend uses a
single ROS executor thread, so snapshots and events need no handoff locks.
Operating-state events are transported separately from replaceable snapshots.

## Runtime configuration and launches

`configuration.py` validates common fields and each backend's supported options.
Unknown runtime and grasp options, invalid booleans, and nonfinite or nonpositive
rates are rejected. Newton supports pose-error grasping; force/torque policies
and constraint tuning belong to PyBullet. The cache option is `generated_root`.
Select the Isaac interpreter through `DVRK_ISAAC_SIM_PYTHON` or `ISAAC_SIM_DIR`.

`launch.py` builds simulator, patient-cart, and OpenXR sessions. Existing package
launch filenames remain thin wrappers. An empty `headless` override uses the
runtime YAML; patient-cart sessions default to headless. `console` selects the
namespace for monitor/startup clients. Simulation scripts always start under
ROS Python and select the engine interpreter through the worker boundary.

JHU cart scenes include a platform's `simulator_cart.yaml` and add backend camera
settings. Identical system JSON files use relative symlinks to a common system
configuration. Exercise aliases include their canonical scene files.
