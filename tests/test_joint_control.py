import math
import unittest

from teleop_core.pose_control import Pose, from_rotvec, norm
from teleop_core.joint_control import JointServoSession, ordered_positions, tracking_twist
from teleop_core.ur3_model import UR3_ARM_JOINTS


class Control:
    def __init__(self):
        self.sent = []
        self.stops = 0
        self.accept = True

    def servoJ(self, q, *args):
        self.sent.append(q)
        return self.accept

    def servoStop(self):
        self.stops += 1


class JointControlTests(unittest.TestCase):
    def setUp(self):
        self.control = Control()
        self.session = JointServoSession(self.control, [1.] * 6, [-math.pi] * 6, [math.pi] * 6)

    def test_feedback_uses_latest_error_and_separate_norm_limits(self):
        actual = Pose((0., 0., 0.), from_rotvec((0., 0., 0.)))
        target = Pose((1., 1., 1.), from_rotvec((0., 0., 1.)))
        linear, angular = tracking_twist(actual, target, 2., 3., .02, .12)
        self.assertAlmostEqual(norm(linear), .02)
        self.assertAlmostEqual(norm(angular), .12)
        self.assertEqual(tracking_twist(target, target, 2., 3., .02, .12), ((0.,) * 3, (0.,) * 3))
        near = Pose((.999, 1., 1.), target.q)
        self.assertAlmostEqual(tracking_twist(near, target, 2., 3., .02, .12)[0][0], .002)

    def test_orientation_wrap_uses_shortest_rotation(self):
        actual = Pose((0.,) * 3, from_rotvec((0., 0., math.pi - .01)))
        target = Pose((0.,) * 3, from_rotvec((0., 0., -math.pi + .01)))
        self.assertAlmostEqual(tracking_twist(actual, target, 1., 1., 1., 1.)[1][2], .02)

    def test_joint_order_and_duplicate_rejection(self):
        result = ordered_positions(list(reversed(UR3_ARM_JOINTS)), list(range(6)), [], [1.] * 6)
        self.assertEqual(result, (5., 4., 3., 2., 1., 0.))
        with self.assertRaises(ValueError):
            ordered_positions([UR3_ARM_JOINTS[0]] * 6, [0.] * 6, [], [1.] * 6)
        with self.assertRaises(ValueError):
            ordered_positions(UR3_ARM_JOINTS, [0.] * 6, [1.1] * 6, [1.] * 6)

    def test_executor_bounds_first_and_subsequent_steps_without_velocity_integration(self):
        actual = (0.,) * 6
        target = (.1, .05, 0., 0., 0., 0.)
        for _ in range(20):
            old = self.session.command or actual
            command = self.session.step(actual, target, .008)
            self.assertLessEqual(max(abs(q - p) for q, p in zip(command, old)), .008 + 1e-12)
            actual = command
        self.assertEqual(command, target)
        self.assertEqual(self.session.step(actual, target, .008), target)

    def test_position_and_tracking_guards_stop(self):
        self.session.step([0.] * 6, [.001] * 6, .008)
        with self.assertRaises(ValueError):
            self.session.step([0.] * 6, [4.] * 6, .008)
        self.assertEqual(self.control.stops, 1)
        self.session.step([0.] * 6, [.001] * 6, .008)
        with self.assertRaises(ValueError):
            self.session.step([.3] * 6, [.301] * 6, .008)
        self.assertEqual(self.control.stops, 2)

    def test_stop_is_idempotent_and_reanchors_to_feedback(self):
        self.session.step([0.] * 6, [.001] * 6, .008)
        self.session.stop()
        self.session.stop()
        self.assertEqual(self.control.stops, 1)
        self.assertEqual(self.session.step([1.] * 6, [1.] * 6, .008), (1.,) * 6)

    def test_nonfinite_and_rejected_commands_stop(self):
        self.control.accept = False
        with self.assertRaises(RuntimeError):
            self.session.step([0.] * 6, [.001] * 6, .008)
        self.assertEqual(self.control.stops, 1)
        self.control.accept = True
        for dt in (0., -1., float('nan')):
            with self.assertRaises(ValueError):
                self.session.step([0.] * 6, [.001] * 6, dt)
        with self.assertRaises(ValueError):
            self.session.step([0.] * 6, [float('nan')] * 6, .008)

    def test_preview_executes_same_checks_without_control(self):
        preview = JointServoSession(None, [1.] * 6, [-math.pi] * 6, [math.pi] * 6)
        self.assertEqual(preview.step([0.] * 6, [.001] * 6, .008), (.001,) * 6)
        self.assertIsNone(preview.step([0.] * 6, None, .008))

    def test_continuous_wrist_unwraps_without_a_full_turn_jump(self):
        session = JointServoSession(None, [1.] * 6, [-math.pi] * 6, [math.pi] * 6,
                                    continuous=(False, False, False, False, False, True))
        actual = [0.] * 5 + [7.]
        target = [0.] * 5 + [7.001 - 2 * math.pi]
        self.assertAlmostEqual(session.step(actual, target, .008)[5], 7.001)

    def test_rearming_stationary_servo_holds_new_actual_instead_of_cached_target(self):
        old = [.01] * 6
        actual = [.05] * 6
        for _ in range(5):
            self.assertEqual(self.session.step(actual, old, .008, stationary=True), tuple(actual))
        moved = self.session.step(actual, [.051] * 6, .008)
        self.assertEqual(moved, (.051,) * 6)

    def test_velocity_reference_tracks_with_a_lagging_position_servo(self):
        actual = (0.,) * 6
        dt = .008
        desired_velocity = .02
        for _ in range(250):
            # Servo's position output remains anchored to measured Q each tick.
            hint = tuple(q + desired_velocity * dt for q in actual)
            command = self.session.step_velocity(actual, hint, [desired_velocity] * 6, dt)
            previous = actual
            actual = tuple(q + dt / .1 * (c - q) for q, c in zip(actual, command))
        measured_velocity = (actual[0] - previous[0]) / dt
        self.assertAlmostEqual(measured_velocity, desired_velocity, places=6)
        self.assertGreater(actual[0], .035)

    def test_velocity_reference_stops_on_frozen_feedback_and_cannot_wind_up(self):
        actual = (0.,) * 6
        count = 0
        with self.assertRaises(ValueError):
            for _ in range(300):
                self.session.step_velocity(actual, [.008] * 6, [1.] * 6, .008)
                count += 1
        self.assertLess(count, 20)
        self.assertIsNone(self.session.command)
        self.assertEqual(self.control.stops, 1)

    def test_velocity_reference_halts_resets_and_reanchors(self):
        command = self.session.step_velocity([0.] * 6, [.001] * 6, [.1] * 6, .008)
        self.assertEqual(self.session.step_velocity(command, command, [0.] * 6, .008), command)
        self.assertIsNone(self.session.step_velocity(command, None, [1.] * 6, .008))
        self.assertEqual(self.session.step_velocity([1.] * 6, [.01] * 6, [0.] * 6, .008), (1.,) * 6)

    def test_velocity_reference_rejects_invalid_inputs_and_joint_bound_crossing(self):
        for velocities in ([float('nan')] * 6, [1.1] * 6, [0.] * 5):
            with self.assertRaises(ValueError):
                self.session.step_velocity([0.] * 6, [0.] * 6, velocities, .008)
        near_bound = [math.pi - .001] * 6
        with self.assertRaises(ValueError):
            self.session.step_velocity(near_bound, near_bound, [1.] * 6, .008)

    def test_velocity_preview_does_not_wind_up_against_a_stationary_robot(self):
        preview = JointServoSession(None, [1.] * 6, [-math.pi] * 6, [math.pi] * 6)
        for _ in range(500):
            command = preview.step_velocity([0.] * 6, [.008] * 6, [1.] * 6, .008)
            self.assertEqual(command, (.008,) * 6)


if __name__ == '__main__':
    unittest.main()
