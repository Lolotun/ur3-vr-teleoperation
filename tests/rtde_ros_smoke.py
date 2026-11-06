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
    def getRuntimeState(self): return 2
    def getSpeedScaling(self): return 1.
    def getTargetSpeedFraction(self): return .78
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
        if shared.get('reject_servoj', False):
            return False
        if shared.get('freeze_plant', False):
            return True  # Feedback is fresh, but the plant cannot follow commands.
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
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetStateValidity
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
                      qd=[0.] * 6, controls=[], freeze=False, freeze_plant=False,
                      reject_servoj=False, stamp=0.)
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
        scene = yaml.safe_load((Path(get_package_share_directory('teleop_core')) /
                                'config/collision_scene.yaml').read_text())
        scene.pop('check_self_collisions', None)  # Consumed by model construction in production launch.
        guard_params = Path(self.temp.name) / 'guard.yaml'
        guard_params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {**self.model, **scene}}}))
        self.guard = subprocess.Popen(['ros2', 'run', 'teleop_collision', 'collision_guard',
                                      '--ros-args', '--params-file', str(guard_params)],
                                     stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def tearDown(self):
        import os
        import signal
        for process in (self.servo, self.guard):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
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
            rearm_pose = None
            if failure == 'servo_halt':
                # Inject the protective event without moving the fake robot
                # into a singularity. Subsequent real Servo status 0 must not
                # restore either the old joint reference or old VR anchor.
                relative.on_servo_status(Int8(data=2))
                bridge.on_servo_status(Int8(data=2))
                pump(.1, True, .08, angle=.2)
                count = len(control.calls)
                pump(.2, True, .08, angle=.2)
                self.assertEqual(len(control.calls), count, 'Held operator resumed after Servo halt')
                self.assertIsNone(relative.core.anchors)
                self.assertTrue(bridge.servo_halt_latched)
                rearm_pose = shared['model'].tcp(shared['q'])
            elif failure in ('servo', 'guard'):
                # Freeze output while tracker and VR heartbeat continue.
                import os, signal
                process = self.servo if failure == 'servo' else self.guard
                os.killpg(process.pid, signal.SIGSTOP)
                pump(.25, True, .02)
                self.assertIn('STALE_COMMAND' if failure == 'servo' else 'COLLISION_GUARD_TIMEOUT', bridge.status)
                count = len(control.calls)
                os.killpg(process.pid, signal.SIGCONT)
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
            if rearm_pose is not None:
                self.assertIsNotNone(tracker.target)
                displacement, rotation = distance(tracker.target, rearm_pose)
                self.assertLess(displacement, .001)
                self.assertLess(rotation, .001)
                self.assertFalse(bridge.servo_halt_latched)
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

    def test_tracking_rejection_keeps_feedback_and_requires_release(self):
        self.check_bridge_rejection(execution_failure=False)

    def test_servoj_rejection_keeps_feedback_and_cannot_rearm(self):
        self.check_bridge_rejection(execution_failure=True)

    def test_servo_halt_discards_reference_and_requires_physical_release(self):
        self.check_bridge_rejection(execution_failure=False, servo_halt=True)

    def check_bridge_rejection(self, execution_failure, servo_halt=False):
        # Feed bounded Servo-shaped commands directly to isolate bridge recovery.
        # The real Servo process is not started for this test.
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectoryPoint
        rclpy.init(args=['--ros-args', '-p', 'robot_ip:=mock', '-p', 'enable_motion:=true'])
        shared['freeze_plant'] = True
        shared['reject_servoj'] = execution_failure
        bridge = UrRtdeBridge()
        source = rclpy.create_node('tracking_rejection_source')
        from rclpy.parameter import Parameter
        bridge.set_parameters([Parameter('publish_diagnostics', value=True)])
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(source)
        active = source.create_publisher(Bool, '/teleop/servo/active', 1)
        operator = source.create_publisher(Bool, '/vr/right_hand/active', 1)
        servo_status = source.create_publisher(Int8, '/teleop/servo/status', 1)
        commands = source.create_publisher(JointTrajectory, '/teleop/servo/joint_trajectory', 1)
        feedback, poses = [], []
        traces = []
        source.create_subscription(String, '/teleop/rtde_diagnostics',
                                   lambda msg: traces.append(json.loads(msg.data)), 1000)
        source.create_subscription(JointState, '/joint_states', feedback.append, 10)
        source.create_subscription(PoseStamped, '/teleop/tcp_actual', poses.append, 10)
        recorder = None
        trace_path = Path(self.temp.name) / 'execution_trace.jsonl'
        if execution_failure:
            recorder = subprocess.Popen(
                [sys.executable, '-m', 'teleop_core.rtde_diagnostics',
                 '--output', str(trace_path), '--duration', '1.5'],
                stdout=self.log, stderr=self.log)

        def pump(seconds, held=False, speed=1., status=0, physical_held=None):
            end = time.monotonic() + seconds
            next_send = 0.
            while time.monotonic() < end:
                if time.monotonic() >= next_send:
                    next_send = time.monotonic() + .008
                    active.publish(Bool(data=held))
                    operator.publish(Bool(data=held if physical_held is None else physical_held))
                    servo_status.publish(Int8(data=status))
                    message = JointTrajectory()
                    message.joint_names = list(UR3_ARM_JOINTS)
                    point = JointTrajectoryPoint()
                    point.positions = list(shared['q'])
                    point.positions[0] += speed * .008
                    point.velocities = [speed, 0., 0., 0., 0., 0.]
                    point.time_from_start.nanosec = 8_000_000
                    message.points = [point]
                    commands.publish(message)
                executor.spin_once(timeout_sec=.001)

        try:
            pump(.4)
            if not execution_failure:
                pump(.8, held=True)
                self.assertEqual(bridge.status, 'TRACKING')
                self.assertTrue(all(abs(q - a) <= .1 + 1e-12
                                    for q, a in zip(shared['controls'][0].calls[-1], shared['q'])))
                self.assertEqual(shared['controls'][0].stops, 0)
                if servo_halt:
                    pump(.08, held=True, status=2)
                else:
                    # Unexpected actual displacement still trips the hard guard;
                    # ordinary integration against a frozen plant no longer does.
                    shared['q'][0] -= .25
            deadline = time.monotonic() + 2
            expected = ('EXECUTION_FAULT_RESTART_REQUIRED' if execution_failure else
                        'SERVO_HALT_RELEASE_TO_REARM' if servo_halt else 'COMMAND_REJECTED_RELEASE_TO_REARM')
            while not bridge.status.startswith(expected) and time.monotonic() < deadline:
                pump(.03, held=True)
            self.assertIn(expected, bridge.status)
            if execution_failure:
                self.assertIn('servoJ rejected: command=', bridge.status)
            elif not servo_halt:
                self.assertIn('Joint tracking error', bridge.status)
                self.assertIn('joint=shoulder_pan_joint', bridge.status)
                self.assertIn('limit=0.150000rad', bridge.status)
            control = shared['controls'][0]
            self.assertGreater(control.stops, 0)
            sent, received, tcp_received = len(control.calls), len(feedback), len(poses)
            pump(.35, held=True)
            self.assertEqual(len(control.calls), sent, 'Held clutch resumed rejected commands')
            self.assertGreater(len(feedback), received + 10)
            self.assertGreater(len(poses), tcp_received + 10)
            self.assertTrue(bridge.worker.is_alive(), bridge.status)
            self.assertLess(time.monotonic() - bridge.state[2], .1)
            self.assertGreater(len(traces), 10)
            sent_traces = [row for row in traces if row['command_sent']]
            if not execution_failure:
                self.assertTrue(sent_traces)
            for row in sent_traces:
                self.assertIn(row['q_cmd'], control.calls)
                self.assertIn(tuple(row['q_actual']), (tuple(DEFAULT_SEED), tuple(shared['q'])))
                self.assertEqual(row['getSpeedScaling'], 1.)
            self.assertTrue(any(row['status'].startswith(expected) for row in traces))
            self.assertTrue(all(row['q_cmd'] is None for row in traces if row['status'].startswith(expected)))
            if servo_halt:
                pump(.1, held=False, speed=0., physical_held=True)
                pump(.1, held=True, speed=0., physical_held=True)
                self.assertEqual(len(control.calls), sent, 'Derived active=false cleared a Servo halt')
            shared['reject_servoj'] = False
            pump(.1, held=False, speed=0.)
            pump(.2, held=True, speed=0.)
            if execution_failure:
                self.assertEqual(len(control.calls), sent, 'Execution fault resumed after clutch release')
                self.assertIn('EXECUTION_FAULT_RESTART_REQUIRED', bridge.status)
                self.assertEqual(recorder.wait(timeout=5), 0)
                events = [json.loads(line) for line in trace_path.read_text().splitlines()]
                self.assertTrue(any(event.get('topic') == '/teleop/rtde_diagnostics' for event in events))
                self.assertTrue(any(event.get('topic') == '/teleop/servo/joint_trajectory' for event in events))
            else:
                self.assertGreater(len(control.calls), sent, bridge.status)
                self.assertEqual(bridge.status, 'TRACKING')
                self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(control.calls[-1], shared['q'])))
        finally:
            if recorder is not None and recorder.poll() is None:
                recorder.terminate()
                recorder.wait(timeout=5)
            executor.shutdown()
            bridge.destroy_node()
            source.destroy_node()
            rclpy.shutdown()
    def test_motion_with_real_servo_and_fake_rtde(self): self.run_mode(True)
    def test_servo_loss_requires_release(self): self.run_mode(True, failure='servo')
    def test_collision_guard_loss_requires_release(self): self.run_mode(True, failure='guard')
    def test_servo_halt_reanchors_vr_target(self): self.run_mode(True, failure='servo_halt')


    def test_real_geometry_floor_and_self_collision(self):
        import random
        rclpy.init()
        node = rclpy.create_node('collision_geometry_test')
        publisher = node.create_publisher(JointState, '/joint_states', 10)
        client = node.create_client(GetStateValidity, '/teleop/check_state_validity')
        scenes = []
        guard_status = []
        node.create_subscription(PlanningScene, '/planning_scene', scenes.append, 10)
        node.create_subscription(String, '/teleop/collision_status', lambda msg: guard_status.append(msg.data), 10)
        q = list(DEFAULT_SEED)

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                message = JointState()
                message.header.stamp = node.get_clock().now().to_msg()
                message.name = list(UR3_ARM_JOINTS)
                message.position = list(q)
                message.velocity = [0.] * 6
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.005)

        def check(position, actual_position=None):
            nonlocal q
            q = list(position if actual_position is None else actual_position)
            pump(.06)
            request = GetStateValidity.Request()
            request.group_name = 'manipulator'
            request.robot_state.joint_state.name = list(UR3_ARM_JOINTS)
            request.robot_state.joint_state.position = list(position)
            future = client.call_async(request)
            end = time.monotonic() + 2
            while not future.done() and time.monotonic() < end:
                pump(.005)
            self.assertTrue(future.done(), 'Collision guard did not respond')
            return future.result()

        try:
            end = time.monotonic() + 10
            while not client.service_is_ready() and time.monotonic() < end:
                self.assertIsNone(self.guard.poll(), 'Collision guard exited')
                pump(.05)
            self.assertTrue(client.service_is_ready())
            pump(1.1)
            self.assertTrue(check(DEFAULT_SEED).valid, 'Expected clear seed pose')
            floor = [obj for scene in scenes for obj in scene.world.collision_objects if obj.id == 'teleop_floor']
            self.assertTrue(floor, 'Floor was not published to planning scene')
            box, pose = floor[-1].primitives[0], floor[-1].primitive_poses[0]
            self.assertAlmostEqual(pose.position.z + box.dimensions[2] / 2, -.01)
            blocked = check([0., math.pi / 2, 0., 0., math.pi / 2, 0.])
            self.assertFalse(blocked.valid)
            self.assertTrue(any('teleop_floor' in (c.contact_body_1, c.contact_body_2) for c in blocked.contacts))
            rng = random.Random(12)
            found_self = None
            for _ in range(100):
                position = [0., rng.uniform(-2.5, -.3), rng.uniform(-2.9, 2.9),
                            rng.uniform(-3., 3.), rng.uniform(-3., 3.), 0.]
                result = check(position)
                if not result.valid and result.contacts and all(
                        'teleop_floor' not in (c.contact_body_1, c.contact_body_2) for c in result.contacts):
                    found_self = result
                    break
            self.assertIsNotNone(found_self, 'Expected a detected self-collision in sampled folded poses')

            # Find the first contact along a one-joint sweep. Bisection leaves
            # a non-intersecting pose just inside the configured clearance band.
            clear = list(DEFAULT_SEED)
            contact = None
            for i in range(1, 35):
                candidate = list(DEFAULT_SEED)
                candidate[1] += .1 * i
                result = check(candidate)
                if not result.valid and result.contacts:
                    contact = candidate
                    break
                self.assertTrue(result.valid, 'Unexpected non-geometric rejection during sweep')
                clear = candidate
            self.assertIsNotNone(contact, 'Expected a contact boundary in shoulder sweep')
            for _ in range(9):
                middle = [(a + b) / 2 for a, b in zip(clear, contact)]
                if check(middle).valid:
                    clear = middle
                else:
                    contact = middle
            near = list(clear)
            self.assertTrue(check(near).valid, 'Holding a non-intersecting close pose must be allowed')
            pump(.01)
            self.assertEqual(guard_status[-1], 'HOLDING_IN_PROXIMITY')
            retreat = list(near)
            retreat[1] -= .02
            self.assertTrue(check(retreat, actual_position=near).valid, 'Retreat must improve an existing small gap')
            pump(.01)
            self.assertEqual(guard_status[-1], 'RETREATING_FROM_PROXIMITY')
            deeper = list(near)
            deeper[1] += .002
            self.assertFalse(check(deeper, actual_position=near).valid, 'Approaching the nearby obstacle must stay blocked')
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
