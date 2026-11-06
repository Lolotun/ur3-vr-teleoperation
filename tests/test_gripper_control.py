import unittest

from teleop_core.gripper_control import LatestGripperCommand, parse_position


class GripperTests(unittest.TestCase):
    def test_legacy_binary_commands_still_work(self):
        for command in ('open', 'release', '0'):
            self.assertEqual(parse_position(command), 0)
        for command in ('close', 'grab', 'grip', '1'):
            self.assertEqual(parse_position(command), 255)

    def test_partial_positions_and_configured_endpoints(self):
        self.assertEqual(parse_position('0.500'), 128)
        self.assertEqual(parse_position('128'), 128)
        self.assertEqual(parse_position('0.500', 10, 210), 110)
        self.assertEqual(parse_position('1.000', 10, 210), 210)
        self.assertEqual(parse_position('0.000', 10, 210), 10)
        self.assertEqual(parse_position('0.250', 210, 10), 160)
        self.assertEqual(parse_position('300'), 255)

    def test_all_unity_quantized_positions_round_trip(self):
        for position in range(256):
            command = f'{position / 255:.3f}'
            self.assertEqual(parse_position(command), position, command)

    def test_nonfinite_and_malformed_inputs_are_rejected(self):
        for command in ('nan', 'inf', '-inf', '0,5', '', 'unknown'):
            self.assertIsNone(parse_position(command))

    def test_bursts_execute_only_latest_position_including_release(self):
        commands = LatestGripperCommand()
        for position in (20, 60, 100, 200, 0):
            commands.offer(position)
        self.assertEqual(commands.take(), 0)
        commands.acknowledge(0)
        commands.offer(0)
        self.assertIsNone(commands.take())

    def test_failed_send_does_not_acknowledge_or_retry_stale_command(self):
        commands = LatestGripperCommand()
        commands.offer(100)
        self.assertEqual(commands.take(), 100)
        self.assertIsNone(commands.take())
        commands.offer(100)
        self.assertEqual(commands.take(), 100)
        commands.acknowledge(100)
        commands.offer(100)
        self.assertIsNone(commands.take())


if __name__ == '__main__':
    unittest.main()
