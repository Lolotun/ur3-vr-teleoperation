"""Run only in an isolated ROS domain: synthetic observations, no robot drivers."""
import argparse
from array import array
import json
import math
import socket
import threading
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from teleop_core.control_mux import ControlMuxNode
from ros_policy_bridge import PolicyBridge, Server, Handler
from lerobot_robot_ur3.protocol import JOINTS, send_packet, recv_packet


class Fixture(Node):
    def __init__(self):
        super().__init__('synthetic_robot')
        self.pubs = {key: self.create_publisher(kind, topic, 10) for key, kind, topic in (
            ('q', JointState, '/joint_states'), ('tcp', PoseStamped, '/teleop/tcp_actual'),
            ('wrist', Image, '/cameras/wrist/image_raw'), ('external', Image, '/cameras/external/image_raw'),
            ('rtde', String, '/teleop/rtde_status'))}
        self.active, self.targets, self.grips, self.predictions = False, [], [], []
        self.gripper_pub = self.create_publisher(String, '/teleop/gripper_state', 10)
        self.gripper_valid = True
        self.subs = [self.create_subscription(kind, topic, cb, 10) for kind, topic, cb in (
            (Bool, '/teleop/active', lambda m: setattr(self, 'active', m.data)),
            (PoseStamped, '/teleop/target_pose', self.targets.append),
            (String, '/teleop/gripper_command', self.grips.append),
            (String, '/policy/prediction', self.predictions.append))]
        self.create_timer(1/30, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        q = JointState()
        q.header.stamp, q.name, q.position = stamp, list(JOINTS), [0.]*6
        self.pubs['q'].publish(q)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'base'
        pose.pose.position.x, pose.pose.position.z, pose.pose.orientation.w = .1, .2, 1.
        self.pubs['tcp'].publish(pose)
        self.pubs['rtde'].publish(String(data='IDLE'))
        self.gripper_pub.publish(String(data=json.dumps({'schema': 1, 'valid': self.gripper_valid,
            'stamp_ns': self.get_clock().now().nanoseconds, 'position': 128, 'position_norm': 128/255,
            'requested_position': 255, 'requested_norm': 1., 'object_status': 2, 'activation_status': 3})))
        image = Image()
        image.header.stamp = stamp
        image.height, image.width, image.encoding, image.step = 480, 640, 'rgb8', 640*3
        image.data = array('B', bytes(480*640*3))
        for key in ('wrist', 'external'):
            self.pubs[key].publish(image)


def wait(predicate, timeout=4):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('timed out')


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--socket', default='/test/policy.sock')
    args.add_argument('--serve-only', action='store_true')
    args = args.parse_args()
    rclpy.init()
    mux, bridge, fixture = ControlMuxNode(), PolicyBridge(True), Fixture()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mux, bridge, fixture):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    Path(args.socket).unlink(missing_ok=True)
    server = Server(args.socket, Handler)
    server.node = bridge
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def select(mode):
        client = fixture.create_client(Trigger, '/teleop/control/'+mode.lower())
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        fixture.destroy_client(client)
        time.sleep(.12)

    def rpc(sock, op, **kwargs):
        send_packet(sock, {'op': op, **kwargs})
        header, buffers = recv_packet(sock)
        assert header['ok'], header
        return header, buffers

    def connect(execute, include_gripper=False):
        sock = socket.socket(socket.AF_UNIX)
        sock.settimeout(4)
        sock.connect(args.socket)
        rpc(sock, 'hello', execute=execute, include_gripper_state=include_gripper)
        return sock

    def command(sock, g=.7, action=None):
        observation, buffers = rpc(sock, 'observe')
        assert len(observation['state']) == 15 and len(buffers) == 2
        return rpc(sock, 'action', observation_id=observation['observation_id'],
                   action=action or [.11, 0., .2, 1., 0., 0., 0., 1., 0., g])

    try:
        time.sleep(1.5)
        bridge.snapshot()
        with connect(False, include_gripper=True) as sock:
            observation, _ = rpc(sock, 'observe')
            assert len(observation['state']) == 16
            assert abs(observation['state'][-1]-128/255) < 1e-6
            assert observation['gripper_feedback']['requested_position'] == 255
            assert observation['gripper_feedback']['object_status'] == 2
            fixture.gripper_valid = False
            time.sleep(.1)
            send_packet(sock, {'op': 'observe'})
            reply, _ = recv_packet(sock)
            assert not reply['ok']
        # Old models continue to work without valid optional gripper feedback.
        assert len(bridge.snapshot()[0]['state']) == 15
        fixture.gripper_valid = True
        if args.serve_only:
            print('FIXTURE READY', flush=True)
            while rclpy.ok():
                time.sleep(.2)
            return
        # Preview produces diagnostics and never a robot command.
        with connect(False) as sock:
            command(sock, action=[.26, 0., .2, 1., 0., 0., 0., 1., 0., .7])
            wait(lambda: bool(fixture.predictions))
            assert mux.core.mode == 'STOP' and not fixture.targets and not fixture.grips
            report = json.loads(fixture.predictions[-1].data)
            assert report['clipped'] and report['reject_reason'] is None
            assert abs(report['limited']['position'][0]-.13) < 1e-6
        select('POLICY')
        with connect(True) as sock:
            command(sock)
            wait(lambda: fixture.active and bool(fixture.grips))
            assert abs(fixture.targets[-1].pose.position.x-.11) < 1e-6
            assert abs(float(fixture.grips[-1].data)-.7) < 1e-6
            # Watchdog latches STOP, later actions cannot rearm it.
            wait(lambda: mux.core.mode == 'STOP')
            assert mux.core.reason == 'POLICY_TIMEOUT'
            observation, _ = rpc(sock, 'observe')
            send_packet(sock, {'op': 'action', 'observation_id': observation['observation_id'],
                              'action': [.11, 0., .2, 1., 0., 0., 0., 1., 0., .7]})
            reply, _ = recv_packet(sock)
            assert not reply['ok']
        select('POLICY')
        sock = connect(True)
        command(sock)
        wait(lambda: mux.core.policy_started)
        rpc(sock, 'stop')
        sock.close()
        wait(lambda: mux.core.mode == 'STOP')
        assert mux.core.reason == 'POLICY_CLIENT_STOP'
        # Even a fresh observation cannot reuse a policy's queued actions in a new epoch.
        select('POLICY')
        with connect(True) as sock:
            command(sock)
            wait(lambda: mux.core.policy_started)
            select('POLICY')
            new_epoch = mux.core.epoch
            observation, _ = rpc(sock, 'observe')
            send_packet(sock, {'op': 'action', 'observation_id': observation['observation_id'],
                              'action': [.11, 0., .2, 1., 0., 0., 0., 1., 0., .7]})
            reply, _ = recv_packet(sock)
            assert not reply['ok']
            time.sleep(.1)
            assert mux.core.mode == 'POLICY' and mux.core.epoch == new_epoch
            assert not mux.core.policy_started
        # Execution clips the raw 16cm goal, and a 21cm goal still latches STOP.
        select('POLICY')
        with connect(True) as sock:
            action = [.26, 0., .2, math.cos(.4), math.sin(.4), 0., -math.sin(.4), math.cos(.4), 0., .7]
            command(sock, action=action)
            wait(lambda: mux.core.policy_started and fixture.active)
            wait(lambda: abs(fixture.targets[-1].pose.position.x-.13) < 1e-6)
            accepted = fixture.targets[-1].pose.orientation
            assert abs(2*math.acos(abs(accepted.w))-math.radians(10)) < 1e-6
            assert abs(float(fixture.grips[-1].data)-.7) < 1e-6
            command(sock, action=[.31, 0., .2, 1., 0., 0., 0., 1., 0., .7])
            wait(lambda: mux.core.mode == 'STOP')
            assert 'distance=21.00cm' in mux.core.reason
        # Invalid orientation is rejected before publishing a command.
        select('POLICY')
        before = len(fixture.targets)
        with connect(True) as sock:
            observation, _ = rpc(sock, 'observe')
            send_packet(sock, {'op': 'action', 'observation_id': observation['observation_id'],
                              'action': [.11, 0., .2]+[0.]*6+[.7]})
            reply, _ = recv_packet(sock)
            assert not reply['ok']
        time.sleep(.1)
        assert len(fixture.targets) == before
        print('PASS: ROS IPC preview, 3cm/10deg clipping, raw rejection, watchdog, disconnect, epoch, invalid rotation', flush=True)
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        for node in (mux, bridge, fixture):
            node.destroy_node()
        rclpy.try_shutdown()
        Path(args.socket).unlink(missing_ok=True)


if __name__ == '__main__':
    main()
