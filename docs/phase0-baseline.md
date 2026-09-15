# Phase 0 baseline

Baseline captured on 2026-09-15 before extracting shared implementation from
`dvrk_isaac_sim`.

## Completed locally

- `python3 -m pytest -q` in `dvrk_isaac_sim`: 18 passed.
- Existing source package version: 0.0.1.
- Existing common-looking modules identified: `config`, `rotations`,
  `cartesian_frames`, `command_validation`, `operating_state`, `ros_messages`,
  `ros_qos`, and `ros_interface`.

## Still required on a configured simulator host

- Capture exact ROS topic names, types, QoS, frame IDs, initial state, and
  default configuration from a running node.
- Run the unmodified `dvrk_arm_test.py` for every currently supported arm and
  retain logs.
- Record joint/pose/Jacobian and ECM-view golden data.
- Retain PyBullet EGL renderer identification and benchmark output.

These gaps must be closed before claiming the Phase 0 exit criteria. The
package scaffold does not change the current Isaac package or its behavior.
