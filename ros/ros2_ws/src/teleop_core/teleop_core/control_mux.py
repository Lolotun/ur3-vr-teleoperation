"""Select exactly one pose/gripper source. Startup and policy faults select STOP."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from teleop_core.control_mux_core import ControlMux
from teleop_core.gripper_control import parse_position
from teleop_core.relative_pose_controller import read_pose, pose_message


class ControlMuxNode(Node):
    def __init__(self):
        super().__init__('control_mux')
        for key, value in {'initial_mode': 'STOP', 'timeout': .25, 'feedback_timeout': .25,
                           'policy_max_distance': .20, 'policy_max_rotation': .5,
                           'policy_target_distance': .03, 'policy_target_rotation': math.radians(10)}.items():
            self.declare_parameter(key, value)
        self.core = ControlMux(timeout=float(self.get_parameter('timeout').value),
            feedback_timeout=float(self.get_parameter('feedback_timeout').value),
            max_distance=float(self.get_parameter('policy_max_distance').value),
            max_rotation=float(self.get_parameter('policy_max_rotation').value),
            target_distance=float(self.get_parameter('policy_target_distance').value),
            target_rotation=float(self.get_parameter('policy_target_rotation').value))
        self.core.switch(self.get_parameter('initial_mode').value.upper(), time.monotonic())
        self.pose_pub = self.create_publisher(PoseStamped, '/teleop/target_pose', 1)
        self.active_pub = self.create_publisher(Bool, '/teleop/active', 1)
        self.grip_pub = self.create_publisher(String, '/teleop/gripper_command', 1)
        self.target_status_pub = self.create_publisher(String, '/policy/target_status', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/teleop/control_mode', qos)
        self.status_pub = self.create_publisher(String, '/teleop/control_status', qos)
        self.create_subscription(PoseStamped, '/vr/target_pose', self.on_vr_pose, 1)
        self.create_subscription(Bool, '/vr/target_active', lambda m: self.core.vr_enable(m.data, time.monotonic()), 1)
        self.create_subscription(Bool, '/vr/right_hand/active', lambda m: self.core.raw_clutch(m.data, time.monotonic()), 1)
        self.create_subscription(String, '/team_1/gripper_command', self.on_grip, 1)
        self.create_subscription(PoseStamped, '/teleop/tcp_actual', self.on_actual, 1)
        self.create_subscription(String, '/policy/command', self.on_policy, 1)
        self.create_subscription(String, '/policy/stop', self.on_policy_stop, 1)
        self.mode_services = [self.create_service(Trigger, '/teleop/control/' + mode.lower(), self.select(mode))
                         for mode in ('STOP', 'VR', 'POLICY')]
        self.create_timer(1/125, self.tick)
        self.create_timer(1/30, self.publish_status)

    def select(self, mode):
        def callback(request, response):
            self.core.switch(mode, time.monotonic())
            self.active_pub.publish(Bool(data=False))
            self.publish_status()
            response.success, response.message = True, f'{mode}, epoch={self.core.epoch}; old goals cleared'
            return response
        return callback

    def checked_pose(self, msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if msg.header.frame_id != 'base' or not 0 <= age <= self.core.timeout:
            raise ValueError('invalid pose frame/time')
        return read_pose(msg)

    def on_actual(self, msg):
        try:
            self.core.feedback(self.checked_pose(msg), time.monotonic())
        except ValueError:
            self.core.actual = None

    def on_vr_pose(self, msg):
        try:
            self.core.vr_pose(self.checked_pose(msg), time.monotonic())
        except ValueError:
            self.core.vr_target, self.core.release_seen = None, False

    def on_grip(self, msg):
        position = parse_position(msg.data)
        if position is not None:
            self.core.vr_grip(position/255.)

    def on_policy(self, msg):
        if self.core.mode != 'POLICY':
            return
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError('policy command must be an object')
            if event.get('epoch') != self.core.epoch:
                return  # Delayed packets cannot stop a newer control session.
            age = (self.get_clock().now().nanoseconds-event['stamp_ns'])/1e9
            if not 0 <= age <= self.core.timeout:
                raise ValueError('stale policy command')
            self.core.policy(event, time.monotonic())
            self.target_status_pub.publish(String(data=json.dumps({
                **self.core.policy_diagnostic, 'accepted': True, 'epoch': self.core.epoch,
                'stamp_ns': self.get_clock().now().nanoseconds})))
        except (ValueError, KeyError, TypeError) as exc:
            self.core.stop('POLICY_REJECTED: ' + str(exc), time.monotonic())
            self.target_status_pub.publish(String(data=json.dumps({
                'accepted': False, 'reason': self.core.reason, 'epoch': self.core.epoch,
                'stamp_ns': self.get_clock().now().nanoseconds})))

    def publish_status(self):
        self.mode_pub.publish(String(data=self.core.mode))
        self.status_pub.publish(String(data=json.dumps({'mode': self.core.mode, 'epoch': self.core.epoch,
            'reason': self.core.reason, 'owner': self.core.policy_owner,
            'policy_limits': self.core.policy_limits(),
            'stamp_ns': self.get_clock().now().nanoseconds})))

    def on_policy_stop(self, msg):
        try:
            event = json.loads(msg.data)
            if (isinstance(event, dict) and self.core.mode == 'POLICY' and event.get('epoch') == self.core.epoch
                    and event.get('session') == self.core.policy_owner):
                self.core.stop('POLICY_CLIENT_STOP', time.monotonic())
        except (ValueError, TypeError):
            pass

    def tick(self):
        old_mode = self.core.mode
        pose, grip = self.core.output(time.monotonic())
        self.active_pub.publish(Bool(data=pose is not None))
        if pose is not None:
            self.pose_pub.publish(pose_message(pose, 'base', self.get_clock().now().to_msg()))
        if grip is not None:
            self.grip_pub.publish(String(data=f'{grip:.6f}'))
        if self.core.mode != old_mode:
            self.publish_status()


def main():
    rclpy.init()
    node = ControlMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
