# dvrk_simulator_base

Shared, simulator-independent contracts and CRTK behavior for dVRK simulation
backends. This package must not import Isaac Sim or PyBullet.

The initial implementation contains validated immutable core types, quaternion
conversions with explicit ROS XYZW ordering, robot configuration, Cartesian
frame utilities, command-message validation, operating state, and the runtime
kinematics protocol. Code is copied from `dvrk_isaac_sim` while the Isaac
package remains unchanged.

URDF parsing and simulator model import are deliberately backend-owned. The
base package describes semantic joint and frame names in configuration and
requires name-based mappings through its backend contracts, but does not build
or interpret a URDF tree.

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
