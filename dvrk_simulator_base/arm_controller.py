"""Engine-independent CRTK state, arm motion, and jaw motion handling."""

import numpy as np

from .cartesian_command import CartesianCommand
from .operating_state import CRTKOperatingState
from .trajectory import JointTrajectory


class ArmController:
    """Keep arm and jaw trajectories independent; engines supply Cartesian IK."""

    def __init__(self, config, commands):
        self.config = config
        self.commands = commands
        self.joint_setpoint = np.array(config.home_position, dtype=float, copy=True)
        self.joint_velocity = np.zeros_like(self.joint_setpoint)
        self.jaw_setpoint = 0.0
        self.jaw_velocity = 0.0
        self.joint_trajectory = None
        self.jaw_trajectory = None
        self.operating_state = CRTKOperatingState(CRTKOperatingState.ENABLED)
        self.operating_state_event_pending = False
        self.move_failure_pending = False
        self.command_warnings = []
        self.lower_limits = np.array([joint.lower for joint in config.joints])
        self.upper_limits = np.array([joint.upper for joint in config.joints])
        self.max_velocities = np.array([joint.velocity for joint in config.joints])
        jaw = config.raw.get("robot", {}).get("jaw", {})
        self.jaw_lower = float(jaw.get("lower", -0.349066))
        self.jaw_upper = float(jaw.get("upper", 1.39626))
        self.jaw_speed = float(jaw.get("velocity", 0.4))
        if not np.isfinite([self.jaw_lower, self.jaw_upper]).all() or self.jaw_lower > self.jaw_upper:
            raise ValueError("jaw limits must be finite and ordered")
        if not np.isfinite(self.jaw_speed) or self.jaw_speed <= 0.0:
            raise ValueError("jaw velocity must be finite and positive")

    def valid_joint_target(self, target):
        return bool(target.shape == self.joint_setpoint.shape
                    and np.isfinite(target).all()
                    and np.all(target >= self.lower_limits - 1e-6)
                    and np.all(target <= self.upper_limits + 1e-6))

    def cancel_motion(self):
        self.joint_trajectory = None
        self.jaw_trajectory = None
        self.joint_velocity.fill(0.0)
        self.jaw_velocity = 0.0

    def reset_motion(self):
        self.cancel_motion()
        self.joint_setpoint = np.array(self.config.home_position, dtype=float, copy=True)
        self.jaw_setpoint = 0.0
        self.move_failure_pending = False

    def advance_commands(self, now, solve_ik, resolve_target, *, measured_position=None):
        """Drain ordered commands, then sample active trajectories at ``now``.

        A measured position lets a rate-limited engine freeze immediately when
        motion is disabled instead of continuing toward its previous target.
        """
        self.move_failure_pending = False
        joint_started = jaw_started = False
        for command in self.commands.drain():
            channel = command.channel
            move = channel in {"move_jp", "move_cp", "jaw/move_jp"}
            if channel == "state_command":
                success, message = self.operating_state.command(command.payload)
                if success:
                    self.operating_state_event_pending = True
                    if not self.operating_state.accepts_motion:
                        self.cancel_motion()
                        if measured_position is not None:
                            self.joint_setpoint = np.array(measured_position, dtype=float, copy=True)
                else:
                    self.command_warnings.append(f"rejected operating state command {command.payload!r}: {message}")
                continue
            if not self.operating_state.accepts_motion:
                self.move_failure_pending |= move
                continue
            if channel in {"servo_jp", "move_jp", "servo_cp", "move_cp"}:
                if channel.endswith("cp"):
                    try:
                        target = command.payload
                        if isinstance(target, CartesianCommand):
                            target = resolve_target(target)
                        result = solve_ik(target, self.joint_setpoint)
                        target = np.asarray(result.position, dtype=float)
                        valid = result.success and self.valid_joint_target(target)
                    except (TypeError, ValueError, AttributeError) as error:
                        self.command_warnings.append(f"rejected {channel}: {error}")
                        self.move_failure_pending |= move
                        continue
                else:
                    target = np.asarray(command.payload, dtype=float)
                    valid = self.valid_joint_target(target)
                if not valid:
                    self.command_warnings.append(f"rejected {channel}: IK failed or joint limits exceeded")
                    self.move_failure_pending |= move
                    continue
                if move:
                    self.joint_trajectory = JointTrajectory(self.joint_setpoint, target, self.max_velocities, now)
                    joint_started = True
                else:
                    self.joint_trajectory = None
                    self.joint_setpoint = target.copy()
                    self.joint_velocity.fill(0.0)
                continue
            if channel in {"jaw/servo_jp", "jaw/move_jp"}:
                target = float(command.payload)
                if not np.isfinite(target) or not self.jaw_lower <= target <= self.jaw_upper:
                    self.move_failure_pending |= move
                    self.command_warnings.append(f"rejected {channel}: jaw limits exceeded")
                    continue
                if move:
                    self.jaw_trajectory = JointTrajectory([self.jaw_setpoint], [target], [self.jaw_speed], now)
                    jaw_started = True
                else:
                    self.jaw_trajectory = None
                    self.jaw_setpoint = target
                    self.jaw_velocity = 0.0
                continue
            self.command_warnings.append(f"unsupported command channel {channel!r}")
        if self.joint_trajectory is not None and not joint_started:
            sample = self.joint_trajectory.sample(now)
            self.joint_setpoint = sample.position.copy()
            self.joint_velocity = sample.velocity.copy()
            if sample.complete:
                self.joint_trajectory = None
        if self.jaw_trajectory is not None and not jaw_started:
            sample = self.jaw_trajectory.sample(now)
            self.jaw_setpoint = float(sample.position[0])
            self.jaw_velocity = float(sample.velocity[0])
            if sample.complete:
                self.jaw_trajectory = None
