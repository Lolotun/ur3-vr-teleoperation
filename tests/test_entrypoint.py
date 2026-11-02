"""Check the actual shell argument vector without starting ROS or a robot."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class EntrypointTests(unittest.TestCase):
    def arguments(self, calibration):
        source = Path(__file__).resolve().parents[1] / 'ros/scripts/entrypoint.sh'
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            # Skip environment setup only; run the production argument-building code.
            script = source.read_text().replace('source /opt/ros/humble/setup.bash', ':')
            script = script.replace('source /ws/install/setup.bash', ':')
            entrypoint = directory / 'entrypoint.sh'
            entrypoint.write_text(script)
            ros2 = directory / 'ros2'
            ros2.write_text('#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
            ros2.chmod(0o755)
            env = {key: value for key, value in os.environ.items() if not key.startswith('RTDE_')}
            env.update(PATH=str(directory) + os.pathsep + env['PATH'], ROBOT_IP='mock')
            if calibration is not None:
                env['RTDE_KINEMATICS_PARAMS_FILE'] = calibration
            result = subprocess.run(['bash', str(entrypoint), 'rtde_real'], env=env,
                                    check=True, capture_output=True, text=True)
            return json.loads(result.stdout)

    def test_missing_or_empty_calibration_uses_launch_default(self):
        for calibration in (None, ''):
            with self.subTest(calibration=calibration):
                args = self.arguments(calibration)
                self.assertEqual(args[:3], ['launch', 'teleop_core', 'rtde_real.launch.py'])
                self.assertFalse(any(arg.startswith('kinematics_params_file:=') for arg in args))
                self.assertFalse(any(arg.endswith(':=') for arg in args))

    def test_explicit_calibration_and_tcp_vectors_are_single_arguments(self):
        args = self.arguments('/robot calibration/ur3.yaml')
        self.assertIn('kinematics_params_file:=/robot calibration/ur3.yaml', args)
        self.assertIn('tcp_xyz:=[0.0, 0.0, 0.0]', args)
        self.assertIn('tcp_rpy:=[0.0, 0.0, 0.0]', args)


if __name__ == '__main__':
    unittest.main()
