# CRTK ROS contract

`dvrk_simulator_base.ros_interface.ArmRosInterface` owns the common ROS 2
surface between simulator backends. Backend runtime threads own model mutation;
ROS callbacks only validate messages and enqueue commands.

Each arm exposes `measured_js`, `setpoint_js`, `measured_cp`, `setpoint_cp`,
`measured_cv`, `operating_state`, `state`, `info`, `warning`, and `error`.
It accepts `servo_jp`, `move_jp`, `servo_cp`, `move_cp`, and `state_command`.
PSMs additionally expose `jaw/measured_js`, `jaw/setpoint_js`,
`jaw/servo_jp`, `jaw/move_jp`, `tool_type`, `local/measured_cp`, and
`local/setpoint_cp`.

Servo commands are superseding depth-one setpoints. Move and state commands
remain ordered in a bounded queue. Invalid commands and queue overflow produce
a warning without mutating backend state. Operating-state and `state` use
transient-local event QoS; `tool_type` is transient-local and latched.

PSM Cartesian topics use the live `ECM_view` frame when an ECM is configured.
PSM `local/*` poses remain in the static configured PSM base frame. Backends
must retain this topic graph, message type, QoS, queueing, and frame behavior;
backend tests should cover their physical command execution separately.