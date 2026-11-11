import unittest
import math
from teleop_core.control_mux_core import ControlMux
from teleop_core.pose_control import Pose, distance, from_rotvec


class ControlMuxTests(unittest.TestCase):
    def setUp(self):
        self.core = ControlMux()
        self.pose = Pose((.1, .0, .2), (0., 0., 0., 1.))

    def command(self, sequence=1, **changes):
        return dict(epoch=self.core.epoch, session='policy-a', sequence=sequence,
                    position=[.11, 0., .2], quaternion=[0., 0., 0., 1.], gripper=.7, **changes)

    def test_stop_and_mode_change_clear_every_source(self):
        c = self.core
        c.feedback(self.pose, .15)
        c.vr_pose(self.pose, .15)
        c.vr_enable(True, .15)
        c.raw_clutch(True, .15)
        self.assertEqual(c.output(.16), (None, None))
        c.switch('VR', .17)
        c.vr_enable(True, .28); c.raw_clutch(True, .28); c.vr_pose(self.pose, .28)
        self.assertIsNone(c.output(.28)[0])  # physical release after selection is mandatory
        c.raw_clutch(False, .29); c.raw_clutch(True, .30)
        self.assertIsNotNone(c.output(.30)[0])
        c.switch('POLICY', .31)
        self.assertEqual(c.output(.42), (None, None))
        c.feedback(self.pose, .42)
        c.policy(self.command(), .42)
        pose, g = c.output(.43)
        self.assertEqual(pose.p, (.11, 0., .2)); self.assertEqual(g, .7)
        c.switch('VR', .44)
        self.assertEqual(c.output(.55), (None, None))

    def test_policy_timeout_latches_stop_and_cannot_rearm_itself(self):
        c = self.core
        c.switch('POLICY', 1.)
        c.feedback(self.pose, 1.11); c.policy(self.command(), 1.11)
        self.assertIsNotNone(c.output(1.12)[0])
        c.feedback(self.pose, 1.4)
        self.assertEqual(c.output(1.4), (None, None))
        self.assertEqual(c.mode, 'STOP')
        self.assertEqual(c.reason, 'POLICY_TIMEOUT')
        with self.assertRaises(ValueError): c.policy(self.command(sequence=2), 1.41)

    def test_bounds_owner_epoch_and_sequence(self):
        c = self.core
        c.switch('POLICY', 1.); c.feedback(self.pose, 1.11)
        event = self.command(); c.policy(event, 1.11)
        for invalid in (event, {**event, 'sequence': 2, 'session': 'other'},
                        {**event, 'sequence': 2, 'epoch': c.epoch-1},
                        {**event, 'sequence': 2, 'position': [.5, 0., .2]},
                        {**event, 'sequence': 2, 'position': [float('nan'), 0., .2]},
                        {**event, 'sequence': 2, 'quaternion': [0., 0., 0., 0.]}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                c.policy(invalid, 1.12)

    def test_vr_gripper_works_with_released_trigger_and_stop_does_not_open(self):
        c = self.core
        c.switch('VR', 0.); c.feedback(self.pose, .11); c.vr_grip(.8)
        self.assertEqual(c.output(.12), (None, .8))
        c.switch('STOP', .13); c.vr_grip(0.)
        self.assertEqual(c.output(.24), (None, None))

    def test_policy_clips_position_and_shortest_rotation_but_keeps_raw_goal(self):
        c = self.core
        c.switch('POLICY', 1.); c.feedback(self.pose, 1.11)
        event = {**self.command(), 'position': [.196, .128, .2],
                 'quaternion': from_rotvec((0., 0., .4))}
        c.policy(event, 1.11)
        pose, g = c.output(1.12)
        self.assertAlmostEqual(pose.p[0], .118)
        self.assertAlmostEqual(pose.p[1], .024)
        translation, angle = distance(self.pose, pose)
        self.assertAlmostEqual(translation, .03)
        self.assertAlmostEqual(angle, math.radians(10))
        self.assertEqual(g, .7)
        self.assertEqual(c.policy_diagnostic['raw']['position'], tuple(event['position']))
        self.assertTrue(c.policy_diagnostic['clipped'])

    def test_original_prediction_is_rejected_before_clipping(self):
        c = self.core
        c.switch('POLICY', 1.); c.feedback(self.pose, 1.11)
        for update in ({'position': [.31, 0., .2]},
                       {'quaternion': from_rotvec((0., 0., .6))},
                       {'position': [.1, 0., -.02]}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                c.policy({**self.command(), **update}, 1.11)
        self.assertFalse(c.policy_started)
        self.assertIsNone(c.grip)

    def test_limits_must_be_finite_positive_and_ordered(self):
        for options in ({'target_distance': float('nan')}, {'max_rotation': 0.},
                        {'target_distance': .3}, {'target_rotation': .6}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ControlMux(**options)

    def test_limited_target_must_also_be_inside_workspace(self):
        c = self.core
        c.switch('POLICY', 1.)
        c.feedback(Pose((.65, 0., .2), self.pose.q), 1.11)
        with self.assertRaisesRegex(ValueError, 'limited policy target outside workspace'):
            c.policy({**self.command(), 'position': [.55,0.,.2]}, 1.11)
        self.assertIsNone(c.policy_owner)
        self.assertFalse(c.policy_started)


if __name__ == '__main__':
    unittest.main(verbosity=2)
