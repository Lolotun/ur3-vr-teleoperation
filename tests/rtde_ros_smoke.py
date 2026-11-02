"""Actual MoveIt Servo + ROS nodes with in-process fake RTDE. No robot connection.

Requires a built teleop_core and the MoveIt/UR description packages. The fake
robot has a first-order position response; this is not a model of UR dynamics.
"""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import types
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros/ros2_ws/src/teleop_core'))

from teleop_core.pose_control import Pose, from_rotvec, mul, inv, rotate, distance
from teleop_core.ur3_model import DEFAULT_SEED, UR3_ARM_JOINTS

shared = {}


def compose(a, b):
    offset = rotate(a.q, b.p)
    return Pose(tuple(x + y for x, y in zip(a.p, offset)), mul(a.q, b.q))


class ForwardModel:
    """Small URDF transform evaluator for fake feedback, independent of Servo IK."""
    def __init__(self, description):
        self.by_child = {}
        for joint in ET.fromstring(description).findall('joint'):
            origin = joint.find('origin')
            xyz = tuple(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else (0.,) * 3
            rpy = tuple(map(float, origin.get('rpy', '0 0 0').split())) if origin is not None else (0.,) * 3
            q = mul(from_rotvec((0., 0., rpy[2])),
                    mul(from_rotvec((0., rpy[1], 0.)), from_rotvec((rpy[0], 0., 0.))))
            axis = joint.find('axis')
            axis = tuple(map(float, axis.get('xyz').split())) if axis is not None else (1., 0., 0.)
            self.by_child[joint.find('child').get('link')] = (
                joint.find('parent').get('link'), Pose(xyz, q), joint.get('name'), axis)

    def tcp(self, joints):
        positions = dict(zip(UR3_ARM_JOINTS, joints))
        def world(link):
            if link not in self.by_child:
                return Pose((0.,) * 3, (0., 0., 0., 1.))
            parent, origin, name, axis = self.by_child[link]
            motion = Pose((0.,) * 3, from_rotvec(tuple(x * positions.get(name, 0.) for x in axis)))
            return compose(compose(world(parent), origin), motion)
        base = world('base')
        tcp = world('teleop_tcp')
        return Pose(rotate(inv(base.q), tuple(x - y for x, y in zip(tcp.p, base.p))), mul(inv(base.q), tcp.q))


class Receive:
    def __init__(self, *args): pass
    def isConnected(self): return True
    def getTimestamp(self):
        if not shared['freeze']:
            shared['stamp'] = time.monotonic()
        return shared['stamp']
    def getActualTCPPose(self): return shared['model'].tcp(shared['q']).ur()
    def getActualQ(self): return list(shared['q'])
    def getActualQd(self): return list(shared['qd'])
    def getSafetyMode(self): return 1
    def getRobotMode(self): return 7
    def disconnect(self): pass


class Control:
    def __init__(self, *args):
        self.calls = []
        self.stops = 0
        shared['controls'].append(self)
    def isConnected(self): return True
    def initPeriod(self): return time.monotonic()
    def waitPeriod(self, start): time.sleep(max(0., .008 - (time.monotonic() - start)))
    def servoJ(self, q, speed, acceleration, dt, lookahead, gain):
        self.calls.append(list(q))
        # A finite position response exposes feedback-reset attenuation that
        # an instantly following fake robot cannot detect.
        alpha = min(1., dt / .1)
        new_q = [b + alpha * (a - b) for a, b in zip(q, shared['q'])]
        shared['qd'] = [(a - b) / dt for a, b in zip(new_q, shared['q'])]
        shared['q'] = new_q
        return True
    def servoStop(self):
        self.stops += 1
        shared['qd'] = [0.] * 6
    def stopScript(self): pass
    def disconnect(self): pass


sys.modules['rtde_receive'] = types.SimpleNamespace(RTDEReceiveInterface=Receive)
sys.modules['rtde_control'] = types.SimpleNamespace(RTDEControlInterface=Control)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String, Int8
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory
from ament_index_python.packages import get_package_share_directory
from teleop_core.relative_pose_controller import RelativePoseController
from teleop_core.pose_tracking_controller import PoseTrackingController
from teleop_core.ur_rtde_bridge import UrRtdeBridge
import yaml


class Integration(unittest.TestCase):
    def setUp(self):
        launch_path = Path(get_package_share_directory('teleop_core')) / 'launch/rtde_real.launch.py'
        spec = importlib.util.spec_from_file_location('rtde_launch', launch_path)
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        self.model, _ = launch.robot_parameters(tcp_xyz=(.0, .0, .1))
        shared.update(model=ForwardModel(self.model['robot_description']), q=list(DEFAULT_SEED),
                      qd=[0.] * 6, controls=[], freeze=False, stamp=0.)
        self.initial = shared['model'].tcp(shared['q'])
        self.temp = tempfile.TemporaryDirectory()
        servo = yaml.safe_load((Path(get_package_share_directory('teleop_core')) /
                                'config/ur3_rtde_servo.yaml').read_text())
        params = Path(self.temp.name) / 'servo.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {**self.model, **servo}}}))
        self.log = (Path(self.temp.name) / 'servo.log').open('w+')
        self.servo = subprocess.Popen(['ros2', 'run', 'moveit_servo', 'servo_node_main',
                                      '--ros-args', '--params-file', str(params)],
                                     stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def tearDown(self):
        import os
        import signal
        if self.servo.poll() is None:
            os.killpg(self.servo.pid, signal.SIGINT)
            try:
                self.servo.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.servo.pid, signal.SIGKILL)
                self.servo.wait(timeout=3)
        self.log.seek(0)
        log = self.log.read()
        if self.servo.returncode not in (0, -2):
            print(log[-6000:])
        self.log.close()
        self.temp.cleanup()

    def run_mode(self, motion, failure='input'):
        rclpy.init(args=['--ros-args', '-p', 'robot_ip:=mock', '-p', f'enable_motion:={str(motion).lower()}'])
        bridge = UrRtdeBridge()
        relative = RelativePoseController()
        tracker = PoseTrackingController()
        source = rclpy.create_node('fake_unity')
        executor = SingleThreadedExecutor()
        for node in (bridge, relative, tracker, source):
            executor.add_node(node)
        poses = source.create_publisher(PoseStamped, '/vr/right_hand/pose', 1)
        active = source.create_publisher(Bool, '/vr/right_hand/active', 1)
        pong = source.create_publisher(String, '/vr/clock_pong', 1)
        client_offset = 113_000_000
        def on_ping(msg):
            reply = json.loads(msg.data)
            reply.update(received_ns=source.get_clock().now().nanoseconds + client_offset,
                         replied_ns=source.get_clock().now().nanoseconds + client_offset, session='fake-quest')
            pong.publish(String(data=json.dumps(reply)))
        source.create_subscription(String, '/teleop/clock_ping', on_ping, 1)
        targets, trajectories, twists, statuses = [], [], [], []
        source.create_subscription(PoseStamped, '/teleop/target_pose', targets.append, 1)
        source.create_subscription(JointTrajectory, '/teleop/servo/joint_trajectory', trajectories.append, 1)
        source.create_subscription(TwistStamped, '/teleop/servo/twist', twists.append, 1)
        source.create_subscription(Int8, '/teleop/servo/status', lambda msg: statuses.append(msg.data), 1)
        start = source.create_client(Trigger, '/servo_node/start_servo')
        def pump(seconds, held=False, x=0., send_pose=True, angle=0.):
            end = time.monotonic() + seconds
            next_send = 0.
            while time.monotonic() < end:
                if time.monotonic() >= next_send:
                    next_send = time.monotonic() + 1 / 30
                    active.publish(Bool(data=held))
                    if send_pose:
                        msg = PoseStamped()
                        msg.header.frame_id = 'vr_robot_base'
                        ns = source.get_clock().now().nanoseconds + client_offset
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
                        msg.pose.orientation.z = math.sin(angle / 2)
                        msg.pose.orientation.w = math.cos(angle / 2)
                        msg.pose.position.x = x
                        poses.publish(msg)
                executor.spin_once(timeout_sec=.001)
        try:
            end = time.monotonic() + 12
            while not start.service_is_ready() and time.monotonic() < end:
                self.assertIsNone(self.servo.poll(), 'Servo exited during startup')
                pump(.05)
            self.assertTrue(start.service_is_ready())
            future = start.call_async(Trigger.Request())
            while not future.done() and time.monotonic() < end:
                pump(.05)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success)
            pump(2.)
            pump(.25, True)
            pump(1.0, True, .02)
            self.assertTrue(targets)
            self.assertTrue(trajectories, 'No output from actual MoveIt Servo')
            self.assertTrue(statuses)
            self.assertAlmostEqual(targets[-1].pose.position.x, self.initial.p[0] - .02, places=4)
            self.assertTrue(any(abs(t.twist.linear.x) > 1e-5 for t in twists))
            if motion:
                control = shared['controls'][0]
                self.assertGreater(len(control.calls), 0, bridge.status)
                self.assertLess(shared['model'].tcp(shared['q']).p[0], self.initial.p[0] - .001)
                self.assertLess(distance(shared['model'].tcp(shared['q']), self.initial)[1], .01)
                for old, new in zip(control.calls, control.calls[1:]):
                    self.assertTrue(all(abs(a - b) <= limit * .008 + 1e-9
                                        for a, b, limit in zip(old, new, bridge.velocity_limits)))
                pump(.6, True, .02, angle=.04)
                self.assertGreater(distance(shared['model'].tcp(shared['q']), self.initial)[1], .002)
            else:
                self.assertFalse(shared['controls'])
                self.assertEqual(shared['q'], DEFAULT_SEED)
            if failure == 'servo':
                # Freeze output while tracker and VR heartbeat continue.
                import os, signal
                os.killpg(self.servo.pid, signal.SIGSTOP)
                pump(.25, True, .02)
                self.assertIn('STALE_COMMAND', bridge.status)
                count = len(control.calls)
                os.killpg(self.servo.pid, signal.SIGCONT)
                pump(.25, True, .02)
                self.assertEqual(count, len(control.calls), 'Servo restart resumed while held')
            else:
                pump(.4, True, .02, False)
                if motion:
                    self.assertGreater(control.stops, 0)
                    count = len(control.calls)
                    pump(.3, True, .02)
                    self.assertEqual(count, len(control.calls), 'Held trigger resumed after stale VR')
            pump(.2, False, .02)
            pump(.3, True, .02)
            if motion:
                self.assertGreater(len(control.calls), count, bridge.status)
            shared['freeze'] = True
            pump(.35, True, .02)
            self.assertIn('feedback stopped', bridge.status)
        finally:
            executor.shutdown()
            bridge.destroy_node()
            relative.destroy_node()
            tracker.destroy_node()
            source.destroy_node()
            rclpy.shutdown()

    def test_preview_with_real_servo(self): self.run_mode(False)
    def test_motion_with_real_servo_and_fake_rtde(self): self.run_mode(True)
    def test_servo_loss_requires_release(self): self.run_mode(True, failure='servo')


if __name__ == '__main__':
    unittest.main()
