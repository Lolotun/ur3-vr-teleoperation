"""ROS-independent pose feedback and bounded execution of Servo commands."""
import math

from teleop_core.pose_control import norm, rotvec, mul, inv
from teleop_core.ur3_model import UR3_ARM_JOINTS

# Humble MoveIt Servo: singularity halt, collision halt, joint bound.
SERVO_HALT_CODES = frozenset((2, 4, 5))


def limited(vector, maximum):
    scale = min(1., maximum / max(norm(vector), 1e-12))
    return tuple(scale * value for value in vector)


def tracking_twist(actual, target, position_gain, rotation_gain, linear_limit, angular_limit):
    linear = tuple(position_gain * (t - a) for t, a in zip(target.p, actual.p))
    angular = tuple(rotation_gain * x for x in rotvec(mul(target.q, inv(actual.q))))
    return limited(linear, linear_limit), limited(angular, angular_limit)


def joint_vector(values):
    result = tuple(float(x) for x in values)
    if len(result) != 6 or not all(math.isfinite(x) for x in result):
        raise ValueError('Expected six finite joint values')
    return result


def ordered_positions(names, positions, velocities, velocity_limits):
    """Reject partial/duplicate commands; never rely on trajectory joint order."""
    if len(names) != 6 or set(names) != set(UR3_ARM_JOINTS):
        raise ValueError('Expected exactly the six UR3 joints')
    positions = joint_vector(positions)
    order = [names.index(name) for name in UR3_ARM_JOINTS]
    if velocities:
        velocities = joint_vector(velocities)
        if any(abs(velocities[j]) > limit + 1e-6 for j, limit in zip(order, velocity_limits)):
            raise ValueError('Servo velocity exceeds execution limit')
    return tuple(positions[j] for j in order)


class JointServoSession:
    """Maintain a continuous servoJ reference with bounded lead over actual Q."""
    def __init__(self, control, velocity_limits, lower_limits, upper_limits,
                 tracking_limit=.15, lookahead=.1, gain=300., continuous=(False,) * 6,
                 lead_soft_start=.06, lead_soft_limit=.10):
        self.control = control
        self.velocity_limits = joint_vector(velocity_limits)
        self.lower_limits = joint_vector(lower_limits)
        self.upper_limits = joint_vector(upper_limits)
        self.continuous = tuple(continuous)
        if len(self.continuous) != 6 or any(type(x) is not bool for x in self.continuous):
            raise ValueError('Expected six continuous-joint flags')
        if any(v <= 0 for v in self.velocity_limits):
            raise ValueError('Joint velocities must be positive')
        if any(lo >= hi for lo, hi in zip(self.lower_limits, self.upper_limits)):
            raise ValueError('Invalid joint position limits')
        if not math.isfinite(tracking_limit) or tracking_limit <= 0:
            raise ValueError('Invalid joint tracking limit')
        self.tracking_limit = tracking_limit
        if not (math.isfinite(lead_soft_start) and math.isfinite(lead_soft_limit)
                and 0 < lead_soft_start < lead_soft_limit < tracking_limit):
            raise ValueError('Expected 0 < lead_soft_start < lead_soft_limit < tracking_limit')
        self.lead_soft_start = lead_soft_start
        self.lead_soft_limit = lead_soft_limit
        self.last_velocity_scale = 1.
        self.lookahead = lookahead
        self.gain = gain
        self.command = None
        self.awaiting_motion = True

    def stop(self):
        self.last_velocity_scale = 0.
        self.awaiting_motion = True
        if self.command is not None:
            self.command = None
            if self.control is not None:
                self.control.servoStop()

    def _check_tracking(self, actual, command, reason):
        errors = tuple(q - a for q, a in zip(command, actual))
        index = max(range(6), key=lambda i: abs(errors[i]))
        if abs(errors[index]) > self.tracking_limit:
            raise ValueError(
                f'{reason}: joint={UR3_ARM_JOINTS[index]} '
                f'actual={actual[index]:.6f}rad command={command[index]:.6f}rad '
                f'delta={errors[index]:+.6f}rad limit={self.tracking_limit:.6f}rad')

    def _integration_scale(self, actual, reference, delta):
        """One scalar preserves the requested joint direction; never move Q by clipping it.

        Only growth of lead is slowed. Reversal toward feedback remains possible.
        An externally increased lead may exceed the soft bound: permit retreat,
        forbid further growth, and leave the independent hard guard in place.
        """
        scale = 1.
        for a, q, d in zip(actual, reference, delta):
            error = q - a
            if d and error * d >= 0:
                x = max(0., min(1., (self.lead_soft_limit - abs(error)) /
                               (self.lead_soft_limit - self.lead_soft_start)))
                scale = min(scale, x * x * (3. - 2. * x))
            if d > 0 and error + d > self.lead_soft_limit:
                scale = min(scale, max(0., (self.lead_soft_limit - error) / d))
            elif d < 0 and error + d < -self.lead_soft_limit:
                scale = min(scale, max(0., (-self.lead_soft_limit - error) / d))
        return scale

    def step_velocity(self, actual, position_hint, velocities, dt):
        """Integrate Servo's limited velocities instead of resetting to actual Q.

        Servo positions are one period ahead of measured Q. Forwarding only
        those positions to a lagging position servo attenuates the requested
        velocity. Preserve a continuous reference; keep Servo's positions as
        a validity check and use measured Q to bound reference tracking error.
        """
        if position_hint is None:
            self.stop()
            return None
        try:
            actual = joint_vector(actual)
            hint = joint_vector(position_hint)
            velocities = joint_vector(velocities)
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError('Invalid joint servo period')
            if self.control is None:
                # Preview has no moving plant. Show a validated single step
                # from actual Q rather than integrating against a stationary
                # robot until a spurious tracking fault stops feedback.
                self.command = None
                self.awaiting_motion = True
            if any(abs(v) > limit + 1e-6 for v, limit in zip(velocities, self.velocity_limits)):
                raise ValueError('Servo velocity exceeds execution limit')
            hint = tuple(a + math.remainder(q - a, 2 * math.pi) if continuous else q
                         for q, a, continuous in zip(hint, actual, self.continuous))
            stationary = all(abs(v) < 1e-12 for v in velocities)
            if any(not continuous and (q < lo or q > hi) for q, lo, hi, continuous in
                   zip(hint, self.lower_limits, self.upper_limits, self.continuous)):
                raise ValueError('Servo position hint outside limits')
            if not (self.awaiting_motion and stationary):
                self._check_tracking(actual, hint, 'Servo position hint too far from feedback')
            reference = self.command if self.command is not None else actual
            self._check_tracking(actual, reference, 'Joint tracking error')
            delta = tuple(v * dt for v in velocities)
            self.last_velocity_scale = self._integration_scale(actual, reference, delta)
            target = tuple(q + self.last_velocity_scale * d for q, d in zip(reference, delta))
            return self.step(actual, target, dt, stationary=stationary)
        except Exception:
            self.stop()
            raise

    def step(self, actual, target, dt, stationary=False):
        if target is None:
            self.stop()
            return None
        try:
            actual, target = joint_vector(actual), joint_vector(target)
            # Servo republishes its previous hold position on a zero Twist.
            # After clutch release the robot may have moved independently;
            # never return to that old position when enabling a stationary goal.
            if self.awaiting_motion and stationary:
                target = self.command if self.command is not None else actual
            elif not stationary:
                self.awaiting_motion = False
            target = tuple(a + math.remainder(q - a, 2 * math.pi) if continuous else q
                           for q, a, continuous in zip(target, actual, self.continuous))
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError('Invalid joint servo period')
            if any(not continuous and (q < lo or q > hi) for q, lo, hi, continuous in
                   zip(target, self.lower_limits, self.upper_limits, self.continuous)):
                raise ValueError('Joint target outside position limits')
            current = self.command if self.command is not None else actual
            self._check_tracking(actual, current, 'Joint tracking error')
            self._check_tracking(actual, target, 'Joint target too far from feedback')
            delta = tuple(q - c for q, c in zip(target, current))
            scale = min([1.] + [v * dt / abs(d) for d, v in zip(delta, self.velocity_limits) if d])
            command = tuple(c + scale * d for c, d in zip(current, delta))
            if any(not continuous and (q < lo or q > hi) for q, lo, hi, continuous in
                   zip(command, self.lower_limits, self.upper_limits, self.continuous)):
                raise ValueError('Joint command outside position limits')
            self.command = command
            if self.control is not None:
                if not self.control.servoJ(list(command), 0., 0., dt, self.lookahead, self.gain):
                    raise RuntimeError(
                        f'servoJ rejected: command={list(command)} dt={dt:.6f}s '
                        f'lookahead={self.lookahead:.6f}s gain={self.gain:g}')
            return command
        except Exception:
            self.stop()
            raise
