"""125 Hz pose-error feedback for the Twist input of MoveIt Servo."""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, String

from teleop_core.joint_control import tracking_twist
from teleop_core.relative_pose_controller import read_pose


class PoseTrackingController(Node):
    def __init__(self):
        super().__init__('pose_tracking_controller')
        defaults = {'frequency': 125., 'command_timeout': .25, 'feedback_timeout': .25,
                    'position_gain': 2., 'rotation_gain': 2., 'max_linear_speed': .02,
                    'max_angular_speed': .12, 'robot_frame': 'base'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in defaults.keys() - {'robot_frame'}:
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key)
        if not 1 <= self.cfg['frequency'] <= 500:
            raise ValueError('frequency outside 1..500')
        self.target = self.actual = None
        self.target_time = self.actual_time = self.active_time = 0.
        self.target_stamp = self.actual_stamp = 0
        self.active = False
        self.arm_time = 0.
        self.faulted = True
        self.released = False
        self.last_status = ''
        self.twist_pub = self.create_publisher(TwistStamped, '/teleop/servo/twist', 1)
        self.active_pub = self.create_publisher(Bool, '/teleop/servo/active', 1)
        self.status_pub = self.create_publisher(String, '/teleop/tracking_status', 1)
        self.create_subscription(PoseStamped, '/teleop/target_pose', self.on_target, 1)
        self.create_subscription(PoseStamped, '/teleop/tcp_actual', self.on_actual, 1)
        self.create_subscription(Bool, '/teleop/active', self.on_active, 1)
        self.create_timer(1 / self.cfg['frequency'], self.tick)

    def on_active(self, msg):
        self.active_time = time.monotonic()
        if not msg.data:
            self.active = False
            self.released = True
            self.target = None
        elif not self.active and self.released:
            self.active = True
            self.arm_time = self.active_time
            self.faulted = False
            self.released = False

    def on_pose(self, msg, name, timeout):
        try:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if msg.header.frame_id != self.cfg['robot_frame'] or not 0 <= age < timeout:
                raise ValueError('Invalid pose frame or age')
            if stamp <= getattr(self, name + '_stamp'):
                raise ValueError('Pose timestamp not increasing')
            setattr(self, name, read_pose(msg))
            setattr(self, name + '_time', time.monotonic())
            setattr(self, name + '_stamp', stamp)
        except ValueError:
            setattr(self, name, None)
            self.faulted = True
            self.released = False

    def on_target(self, msg):
        # Targets received while disabled cannot become a command after rearming.
        if self.active and not self.faulted:
            self.on_pose(msg, 'target', self.cfg['command_timeout'])

    def on_actual(self, msg):
        self.on_pose(msg, 'actual', self.cfg['feedback_timeout'])

    def tick(self):
        now = time.monotonic()
        fresh = (self.target is not None and self.actual is not None
                 and now - self.target_time < self.cfg['command_timeout']
                 and now - self.actual_time < self.cfg['feedback_timeout']
                 and now - self.active_time < self.cfg['command_timeout'])
        enabled = self.active and not self.faulted and fresh
        # Allow a short startup window for the first target after activation.
        stale = ((self.target is not None and now - self.target_time >= self.cfg['command_timeout'])
                 or (self.target is None and now - self.arm_time >= self.cfg['command_timeout'])
                 or (self.actual is None and now - self.arm_time >= self.cfg['feedback_timeout'])
                 or (self.actual is not None and now - self.actual_time >= self.cfg['feedback_timeout'])
                 or now - self.active_time >= self.cfg['command_timeout'])
        if self.active and stale:
            self.faulted = True
            self.released = False
            enabled = False
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['robot_frame']
        if enabled:
            linear, angular = tracking_twist(
                self.actual, self.target, self.cfg['position_gain'], self.cfg['rotation_gain'],
                self.cfg['max_linear_speed'], self.cfg['max_angular_speed'])
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self.twist_pub.publish(msg)
        self.active_pub.publish(Bool(data=bool(enabled)))
        status = 'TRACKING' if enabled else ('RELEASE_TO_REARM' if self.faulted else 'IDLE')
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            self.last_status = status


def main():
    rclpy.init()
    node = PoseTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
