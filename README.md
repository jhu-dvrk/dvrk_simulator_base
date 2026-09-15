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
