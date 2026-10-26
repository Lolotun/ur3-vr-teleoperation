"""Single-thread-owned RTDE interfaces with read-only preview and release-to-rearm."""
import math
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from teleop_core.pose_control import Pose, ServoSession, distance
from teleop_core.relative_pose_controller import read_pose, pose_message
from teleop_core.ur3_model import UR3_ARM_JOINTS


class UrRtdeBridge(Node):
    def __init__(self):
        super().__init__('ur_rtde_bridge')
        defaults={'robot_ip':'','enable_motion':False,'frequency':125.,'command_timeout':.25,
                  'feedback_timeout':.25,'max_linear_speed':.02,'max_angular_speed':.12,
                  'max_tracking_error':.05,'max_angular_tracking_error':.35,
                  'lookahead_time':.1,'gain':300.,'robot_frame':'base'}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        if not self.cfg['robot_ip']: raise ValueError('robot_ip required')
        for k in ['frequency','command_timeout','feedback_timeout','max_linear_speed',
                  'max_angular_speed','max_tracking_error','max_angular_tracking_error']:
            if not math.isfinite(self.cfg[k]) or self.cfg[k]<=0: raise ValueError(k)
        if not 1<=self.cfg['frequency']<=500: raise ValueError('frequency outside 1..500')
        if not .03<=self.cfg['lookahead_time']<=.2 or not 100<=self.cfg['gain']<=2000:
            raise ValueError('Invalid servo tuning')
        self.lock=threading.Lock();self.stopping=threading.Event()
        self.target=None;self.target_time=0.;self.active=False;self.active_time=0.
        self.released=False;self.faulted=True;self.arm_time=0.;self.state=None;self.status='CONNECTING'
        self.create_subscription(PoseStamped,'/teleop/target_pose',self.on_target,1)
        self.create_subscription(Bool,'/teleop/active',self.on_active,1)
        self.pose_pub=self.create_publisher(PoseStamped,'/teleop/tcp_actual',1)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',1)
        self.status_pub=self.create_publisher(String,'/teleop/rtde_status',1)
        self.create_timer(1/30,self.publish_state)
        self.worker=threading.Thread(target=self.run,name='rtde-control',daemon=True)
        self.worker.start()

    def on_target(self,msg):
        try:
            if msg.header.frame_id!=self.cfg['robot_frame']: raise ValueError('target frame')
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if not 0<=age<self.cfg['command_timeout']: raise ValueError('target stamp')
            target=read_pose(msg)
            with self.lock:
                if self.active and not self.faulted:
                    self.target=target;self.target_time=time.monotonic()
        except ValueError: self.trip('INVALID_TARGET_RELEASE_TO_REARM')

    def on_active(self,msg):
        with self.lock:
            now=time.monotonic();self.active_time=now
            if not msg.data:
                self.active=False;self.target=None;self.released=True
            elif not self.active:
                if self.released:
                    self.active=True;self.faulted=False;self.released=False
                    self.arm_time=now;self.target=None

    def trip(self,status):
        with self.lock:
            self.faulted=True;self.released=False;self.active=False;self.target=None;self.status=status

    def publish_state(self):
        with self.lock: state=self.state;status=self.status
        self.status_pub.publish(String(data=status))
        if state is None or time.monotonic()-state[2]>self.cfg['feedback_timeout']: return
        pose,joints,_,stamp=state
        self.pose_pub.publish(pose_message(pose,self.cfg['robot_frame'],stamp))
        msg=JointState();msg.header.stamp=stamp;msg.name=list(UR3_ARM_JOINTS);msg.position=joints
        self.joint_pub.publish(msg)

    def run(self):
        control=None;receive=None;session=None
        try:
            from rtde_receive import RTDEReceiveInterface
            receive=RTDEReceiveInterface(self.cfg['robot_ip'],self.cfg['frequency'])
            if self.cfg['enable_motion']:
                from rtde_control import RTDEControlInterface
                control=RTDEControlInterface(self.cfg['robot_ip'],self.cfg['frequency'])
            session=ServoSession(control,self.cfg['max_linear_speed'],self.cfg['max_angular_speed'],
                                 self.cfg['lookahead_time'],self.cfg['gain'])
            dt=1/self.cfg['frequency'];last_stamp=None;last_fresh=time.monotonic();last_cycle=None
            while not self.stopping.is_set():
                cycle=time.monotonic()
                period=control.initPeriod() if control else None
                if not receive.isConnected() or (control and not control.isConnected()):
                    raise RuntimeError('RTDE disconnected')
                stamp=receive.getTimestamp()
                if not math.isfinite(stamp): raise RuntimeError('Invalid RTDE timestamp')
                if stamp!=last_stamp:
                    last_stamp=stamp;last_fresh=cycle
                    actual=Pose.from_ur(receive.getActualTCPPose());joints=list(receive.getActualQ())
                    if len(joints)!=6 or not all(math.isfinite(x) for x in joints):
                        raise RuntimeError('Invalid joint feedback')
                    with self.lock:
                        self.state=(actual,joints,cycle,self.get_clock().now().to_msg())
                if cycle-last_fresh>self.cfg['feedback_timeout']:
                    raise RuntimeError('RTDE feedback stopped updating')
                with self.lock:
                    active=self.active;target=self.target;age=cycle-self.target_time
                    heartbeat_age=cycle-self.active_time;arm_age=cycle-self.arm_time
                ready=(receive.getSafetyMode()==1 and receive.getRobotMode()==7)
                if control and not ready:
                    session.stop();self.trip('ROBOT_NOT_READY_RELEASE_TO_REARM');target=None
                elif active and (heartbeat_age>self.cfg['command_timeout'] or
                       (target is None and arm_age>self.cfg['command_timeout']) or
                       (target is not None and age>self.cfg['command_timeout'])):
                    session.stop();self.trip('STALE_COMMAND_RELEASE_TO_REARM');target=None
                elif not active: target=None
                if control and target is not None and last_cycle is not None and cycle-last_cycle>4*dt:
                    session.stop();self.trip('MISSED_CONTROL_PERIOD_RELEASE_TO_REARM');target=None
                if session.command is not None:
                    linear,angular=distance(actual,session.command)
                    if linear>self.cfg['max_tracking_error'] or angular>self.cfg['max_angular_tracking_error']:
                        session.stop();self.trip('TRACKING_ERROR_RELEASE_TO_REARM');target=None
                session.step(actual,target,dt)
                with self.lock:
                    if not self.faulted:
                        self.status=('TRACKING' if control else 'PREVIEW') if target else 'IDLE'
                    elif not active and self.released: self.status='READY_RELEASED' if control else 'PREVIEW_RELEASED'
                last_cycle=cycle
                if control: control.waitPeriod(period)
                else: self.stopping.wait(max(0.,dt-(time.monotonic()-cycle)))
        except Exception as exc:
            self.trip('FAULT: '+str(exc));self.get_logger().error(str(exc))
        finally:
            if session:
                try: session.stop()
                except Exception as exc: self.get_logger().error('Stop failed: '+str(exc))
            if control:
                try: control.stopScript()
                except Exception: pass
                try: control.disconnect()
                except Exception: pass
            if receive:
                try: receive.disconnect()
                except Exception: pass

    def destroy_node(self):
        self.stopping.set();self.worker.join(timeout=5.)
        if self.worker.is_alive(): self.get_logger().error('RTDE worker did not stop within 5 seconds')
        return super().destroy_node()


def main():
    rclpy.init();node=UrRtdeBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node();rclpy.shutdown()
