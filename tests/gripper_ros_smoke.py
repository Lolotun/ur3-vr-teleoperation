"""ROS -> fake Robotiq socket test. Never connects to a real gripper."""
import socket
import threading
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from teleop_core.robotiq_gripper_bridge import RobotiqGripperBridge


class GripperIntegration(unittest.TestCase):
    def test_partial_grip_coalescing_and_release(self):
        received = []
        stopping = threading.Event()
        server = socket.socket()
        server.bind(('127.0.0.1', 0))
        server.listen()
        server.settimeout(.1)
        port = server.getsockname()[1]

        def receive():
            while not stopping.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.settimeout(1.)
                    with connection.makefile('r') as lines:
                        for line in lines:
                            if line.startswith('SET POS '):
                                received.append((time.monotonic(), int(line.split()[2])))

        worker = threading.Thread(target=receive, daemon=True)
        worker.start()
        rclpy.init(args=['--ros-args', '-p', 'robot_ip:=127.0.0.1', '-p', f'robot_port:={port}',
                        '-p', 'min_command_interval_sec:=0.1'])
        bridge = RobotiqGripperBridge()
        source = rclpy.create_node('fake_gripper_input')
        publisher = source.create_publisher(String, '/team_1/gripper_command', 1)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(source)

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.005)

        try:
            pump(.3)
            publisher.publish(String(data='0.500'))
            pump(.3)
            self.assertEqual(received[-1][1], 128)
            # Changes within one interval collapse to the most recent request.
            before = len(received)
            for command in ('0.100', '0.300', '0.700', '0.000'):
                bridge._on_command(String(data=command))
            pump(.3)
            self.assertEqual([position for _, position in received[before:]], [0])
            before = len(received)
            publisher.publish(String(data='nan'))
            publisher.publish(String(data='0.000'))
            pump(.3)
            self.assertEqual(len(received), before)
            for a, b in zip(received, received[1:]):
                self.assertGreaterEqual(b[0] - a[0], .08)
        finally:
            executor.shutdown()
            bridge.destroy_node()
            source.destroy_node()
            rclpy.try_shutdown()
            stopping.set()
            worker.join(timeout=2.)
            server.close()


if __name__ == '__main__':
    unittest.main()
