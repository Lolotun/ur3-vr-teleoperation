"""Single-thread-owned RTDE interfaces with read-only preview and release-to-rearm."""
import math
import faulthandler
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from teleop_core.pose_control import Pose
from teleop_core.joint_control import JointServoSession, ordered_positions, joint_vector
from teleop_core.relative_pose_controller import pose_message
from teleop_core.ur3_model import UR3_ARM_JOINTS


class UrRtdeBridge(Node):
    def __init__(self):
        super().__init__('ur_rtde_bridge')
        defaults={'robot_ip':'','enable_motion':False,'frequency':125.,'command_timeout':.25,
                  'feedback_timeout':.25,'trajectory_timeout':.1,'max_joint_tracking_error':.15,
                  'lookahead_time':.1,'gain':300.,'robot_frame':'base',
                  'joint_velocity_limits':[1.,1.,1.,1.2,1.2,1.2],
                  'joint_continuous':[False,False,False,False,False,True],
                  'joint_lower_limits':[-2*math.pi]*6,'joint_upper_limits':[2*math.pi]*6}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        if not self.cfg['robot_ip']: raise ValueError('robot_ip required')
        for k in ['frequency','command_timeout','feedback_timeout','trajectory_timeout','max_joint_tracking_error']:
            if not math.isfinite(self.cfg[k]) or self.cfg[k]<=0: raise ValueError(k)
        if not 1<=self.cfg['frequency']<=500: raise ValueError('frequency outside 1..500')
        if not .03<=self.cfg['lookahead_time']<=.2 or not 100<=self.cfg['gain']<=2000:
            raise ValueError('Invalid servo tuning')
        self.velocity_limits=joint_vector(self.cfg['joint_velocity_limits'])
        # Validate all execution limits before opening any RTDE connection.
        JointServoSession(None,self.velocity_limits,self.cfg['joint_lower_limits'],
                          self.cfg['joint_upper_limits'],self.cfg['max_joint_tracking_error'],
                          continuous=self.cfg['joint_continuous'])
        self.lock=threading.Lock();self.stopping=threading.Event()
        self.target=None;self.target_time=0.;self.target_stamp=0;self.target_deadline=0.
        self.target_velocity=(0.,)*6
        self.active=False;self.active_time=0.;self.state_sequence=0;self.published_sequence=0
        self.released=False;self.faulted=True;self.arm_time=0.;self.state=None;self.status='CONNECTING'
        self.create_subscription(JointTrajectory,'/teleop/servo/joint_trajectory',self.on_trajectory,1)
        self.create_subscription(Bool,'/teleop/servo/active',self.on_active,1)
        self.pose_pub=self.create_publisher(PoseStamped,'/teleop/tcp_actual',1)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',1)
        self.status_pub=self.create_publisher(String,'/teleop/rtde_status',1)
        self.create_timer(1/self.cfg['frequency'],self.publish_state)
        self.worker=threading.Thread(target=self.run,name='rtde-control',daemon=True)
        self.worker.start()

    def on_trajectory(self,msg):
        try:
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            # Humble Servo uses stamp=0 ("execute immediately"). In that case
            # freshness is bounded by receipt time, depth=1 and upstream heartbeat.
            age=(self.get_clock().now().nanoseconds-stamp)/1e9 if stamp else 0.
            if not 0<=age<self.cfg['trajectory_timeout']:
                raise ValueError('trajectory stamp')
            # Servo's non-Gazebo output is one short-horizon setpoint, not a path queue.
            if len(msg.points)!=1: raise ValueError('Expected one Servo trajectory point')
            point=msg.points[0]
            if len(point.velocities)!=6: raise ValueError('Expected Servo joint velocities')
            horizon=point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if not 0<horizon<=2/self.cfg['frequency']+1e-6:
                raise ValueError('Unexpected trajectory horizon')
            target=ordered_positions(list(msg.joint_names),point.positions,point.velocities,self.velocity_limits)
            with self.lock:
                if not self.active or self.faulted: return
                if stamp and stamp<=self.target_stamp: return
                self.target=target;self.target_time=time.monotonic();self.target_stamp=stamp
                self.target_velocity=tuple(point.velocities[list(msg.joint_names).index(name)]
                                           for name in UR3_ARM_JOINTS)
                # Account for transport age; receipt never refreshes an old command.
                self.target_deadline=self.target_time+self.cfg['trajectory_timeout']-age
        except (ValueError,TypeError,OverflowError):
            self.trip('INVALID_TRAJECTORY_RELEASE_TO_REARM')

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
        with self.lock:
            state=self.state;status=self.status;sequence=self.state_sequence
        self.status_pub.publish(String(data=status))
        if state is None or sequence==self.published_sequence or time.monotonic()-state[2]>self.cfg['feedback_timeout']: return
        self.published_sequence=sequence
        pose,joints,_,stamp,velocities=state
        self.pose_pub.publish(pose_message(pose,self.cfg['robot_frame'],stamp))
        msg=JointState();msg.header.stamp=stamp;msg.name=list(UR3_ARM_JOINTS);msg.position=joints;msg.velocity=velocities
        self.joint_pub.publish(msg)

    def run(self):
        control=None;receive=None;session=None
        try:
            from rtde_receive import RTDEReceiveInterface
            self.get_logger().info('Connecting RTDE Receive (read-only feedback)')
            receive=RTDEReceiveInterface(self.cfg['robot_ip'],self.cfg['frequency'])
            self.get_logger().info('RTDE Receive connected')
            if self.cfg['enable_motion']:
                from rtde_control import RTDEControlInterface
                with self.lock: self.status='CONNECTING_CONTROL'
                self.get_logger().info('Connecting RTDE Control (uploading control script)')
                control=RTDEControlInterface(self.cfg['robot_ip'],self.cfg['frequency'])
                self.get_logger().info('RTDE Control connected')
            session=JointServoSession(control,self.velocity_limits,self.cfg['joint_lower_limits'],
                                      self.cfg['joint_upper_limits'],self.cfg['max_joint_tracking_error'],
                                      self.cfg['lookahead_time'],self.cfg['gain'],self.cfg['joint_continuous'])
            dt=1/self.cfg['frequency'];last_stamp=None;last_fresh=time.monotonic();last_cycle=None
            missed_cycle_limit=4*dt
            previous_timings={}
            self.get_logger().info(
                f'RTDE servoJ: {self.cfg["frequency"]:.1f}Hz; '
                f'joint velocity limits={self.velocity_limits}; preview={control is None}')
            while not self.stopping.is_set():
                cycle=time.monotonic()
                timings={}
                period=control.initPeriod() if control else None
                timings['initPeriod']=time.monotonic()-cycle
                started=time.monotonic()
                if not receive.isConnected() or (control and not control.isConnected()):
                    raise RuntimeError('RTDE disconnected')
                stamp=receive.getTimestamp()
                if not math.isfinite(stamp): raise RuntimeError('Invalid RTDE timestamp')
                if stamp!=last_stamp:
                    last_stamp=stamp;last_fresh=cycle
                    actual=Pose.from_ur(receive.getActualTCPPose());joints=list(joint_vector(receive.getActualQ()))
                    velocities=list(joint_vector(receive.getActualQd()))
                    if len(joints)!=6 or not all(math.isfinite(x) for x in joints):
                        raise RuntimeError('Invalid joint feedback')
                    with self.lock:
                        self.state=(actual,joints,cycle,self.get_clock().now().to_msg(),velocities)
                        self.state_sequence+=1
                if cycle-last_fresh>self.cfg['feedback_timeout']:
                    raise RuntimeError('RTDE feedback stopped updating')
                with self.lock:
                    active=self.active;target=self.target;age=cycle-self.target_time
                    heartbeat_age=cycle-self.active_time;arm_age=cycle-self.arm_time
                    deadline=self.target_deadline
                    target_velocity=self.target_velocity
                ready=(receive.getSafetyMode()==1 and receive.getRobotMode()==7)
                timings['feedback_and_state']=time.monotonic()-started
                if control and not ready:
                    session.stop();self.trip('ROBOT_NOT_READY_RELEASE_TO_REARM');target=None
                elif active and (heartbeat_age>self.cfg['command_timeout'] or
                       (target is None and arm_age>self.cfg['command_timeout']) or
                       (target is not None and cycle>deadline)):
                    session.stop();self.trip('STALE_COMMAND_RELEASE_TO_REARM');target=None
                elif not active: target=None
                if control and target is not None and last_cycle is not None and cycle-last_cycle>missed_cycle_limit:
                    elapsed=cycle-last_cycle
                    session.stop();self.trip('MISSED_CONTROL_PERIOD_RELEASE_TO_REARM');target=None
                    details=', '.join(f'{k}={v*1000:.2f}ms' for k,v in previous_timings.items())
                    current=', '.join(f'{k}={v*1000:.2f}ms' for k,v in timings.items())
                    self.get_logger().error(
                        f'RTDE missed cycle: interval={elapsed*1000:.2f}ms, '
                        f'limit={missed_cycle_limit*1000:.2f}ms; previous cycle: {details}; '
                        f'current cycle before stop: {current}; '
                        f'target_age={age*1000:.2f}ms, heartbeat_age={heartbeat_age*1000:.2f}ms')
                started=time.monotonic()
                session.step_velocity(joints,target,target_velocity,dt)
                timings['session_step_total']=time.monotonic()-started
                with self.lock:
                    if not self.faulted:
                        self.status=('TRACKING' if control else 'PREVIEW') if target else 'IDLE'
                    elif not active and self.released: self.status='READY_RELEASED' if control else 'PREVIEW_RELEASED'
                last_cycle=cycle
                timings['before_wait_total']=time.monotonic()-cycle
                started=time.monotonic()
                if control: control.waitPeriod(period)
                else: self.stopping.wait(max(0.,dt-(time.monotonic()-cycle)))
                timings['waitPeriod' if control else 'preview_wait']=time.monotonic()-started
                timings['cycle_total']=time.monotonic()-cycle
                previous_timings=timings
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
    # Native ur_rtde faults bypass Python exception handling. Keep the active
    # Python thread stacks in container logs to identify the failing call.
    faulthandler.enable(all_threads=True)
    rclpy.init();node=UrRtdeBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node();rclpy.try_shutdown()
