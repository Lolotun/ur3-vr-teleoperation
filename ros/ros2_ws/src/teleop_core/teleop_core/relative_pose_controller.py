"""Map held VR hand poses to robot TCP targets with explicit re-anchoring."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from teleop_core.pose_control import Pose, RelativeController, from_rotvec


def read_pose(msg):
    p,q=msg.pose.position,msg.pose.orientation
    return Pose((p.x,p.y,p.z),(q.x,q.y,q.z,q.w))


def pose_message(pose, frame, stamp):
    msg=PoseStamped();msg.header.frame_id=frame;msg.header.stamp=stamp
    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=pose.p
    msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=pose.q
    return msg


class RelativePoseController(Node):
    def __init__(self):
        super().__init__('relative_pose_controller')
        defaults={'input_timeout':.25,'feedback_timeout':.25,'position_scale':1.,
                  'mapping_yaw_deg':180.,'max_displacement':.25,'max_rotation':1.,
                  'source_timeout':1.0,'hand_frame':'vr_robot_base','robot_frame':'base'}
        for name,value in defaults.items(): self.declare_parameter(name,value)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        for key in ['input_timeout','feedback_timeout','position_scale','max_displacement','max_rotation']:
            if not math.isfinite(self.cfg[key]) or self.cfg[key]<=0: raise ValueError(key)
        self.core=RelativeController(from_rotvec((0.,0.,math.radians(self.cfg['mapping_yaw_deg']))),
              self.cfg['position_scale'],self.cfg['max_displacement'],self.cfg['max_rotation'])
        self.hand=None;self.actual=None;self.hand_time=0.;self.actual_time=0.;self.active_time=0.
        self.active=False;self.last_status='';self.last_hand_stamp=0
        self.hand_error='no hand pose received';self.actual_error='no TCP pose received'
        self.diagnostic_times={}
        self.target_pub=self.create_publisher(PoseStamped,'/teleop/target_pose',1)
        self.enabled_pub=self.create_publisher(Bool,'/teleop/active',1)
        self.status_pub=self.create_publisher(String,'/teleop/pose_status',1)
        self.create_subscription(PoseStamped,'/vr/right_hand/pose',self.on_hand,1)
        self.create_subscription(PoseStamped,'/teleop/tcp_actual',self.on_actual,1)
        self.create_subscription(Bool,'/vr/right_hand/active',self.on_active,1)
        self.create_timer(1/30,self.tick)

    def warn_input(self,key,message):
        now=time.monotonic()
        if now-self.diagnostic_times.get(key,float('-inf'))>=2.:
            self.get_logger().warning(message)
            self.diagnostic_times[key]=now

    def on_hand(self,msg):
        try:
            if msg.header.frame_id!=self.cfg['hand_frame']:
                raise ValueError(f"hand frame={msg.header.frame_id!r}, expected={self.cfg['hand_frame']!r}")
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp<=self.last_hand_stamp:
                raise ValueError(f'hand stamp not increasing: stamp={stamp}, last={self.last_hand_stamp}')
            if not -.1<=age<=self.cfg['source_timeout']:
                raise ValueError(f"hand age={age:+.3f}s, allowed=[-0.100, {self.cfg['source_timeout']:.3f}]s; check Quest clock/delivery delay")
            self.last_hand_stamp=stamp
            self.hand=read_pose(msg);self.hand_time=time.monotonic()
            self.hand_error=''
        except ValueError as exc:
            self.hand=None;self.core.fault();self.hand_error=str(exc)
            self.warn_input('hand','Rejected /vr/right_hand/pose: '+self.hand_error)

    def on_actual(self,msg):
        try:
            if msg.header.frame_id!=self.cfg['robot_frame']:
                raise ValueError(f"TCP frame={msg.header.frame_id!r}, expected={self.cfg['robot_frame']!r}")
            # Feedback originates on this ROS host. Reject delayed samples as well as receipt timeouts.
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if not 0<=age<=self.cfg['feedback_timeout']:
                raise ValueError(f"TCP age={age:+.3f}s, allowed=[0, {self.cfg['feedback_timeout']:.3f}]s")
            self.actual=read_pose(msg);self.actual_time=time.monotonic()
            self.actual_error=''
        except ValueError as exc:
            self.actual=None;self.core.fault();self.actual_error=str(exc)
            self.warn_input('actual','Rejected /teleop/tcp_actual: '+self.actual_error)

    def on_active(self,msg):
        self.active=msg.data;self.active_time=time.monotonic()
        if not msg.data: self.core.release()

    def tick(self):
        now=time.monotonic();target=None;status='RELEASED'
        if self.active:
            fresh=(self.hand is not None and self.actual is not None
                   and now-self.hand_time<self.cfg['input_timeout']
                   and now-self.active_time<self.cfg['input_timeout']
                   and now-self.actual_time<self.cfg['feedback_timeout'])
            if fresh:
                target=self.core.update(self.hand,self.actual)
                status='TRACKING' if target is not None else 'RELEASE_TO_REARM'
            else:
                self.core.fault();status='STALE_RELEASE_TO_REARM'
                reasons=[]
                if self.hand is None: reasons.append(self.hand_error)
                if self.actual is None: reasons.append(self.actual_error)
                for label,received,limit in [
                    ('hand',self.hand_time,self.cfg['input_timeout']),
                    ('Trigger heartbeat',self.active_time,self.cfg['input_timeout']),
                    ('TCP',self.actual_time,self.cfg['feedback_timeout'])]:
                    if now-received>=limit:
                        reasons.append(f'{label} receipt age={now-received:.3f}s >= {limit:.3f}s')
                self.warn_input('stale','Target blocked: '+'; '.join(reasons))
        elif now-self.active_time>self.cfg['input_timeout']:
            self.core.fault();status='WAITING_FOR_INPUT'
        self.enabled_pub.publish(Bool(data=target is not None))
        if target is not None:
            self.target_pub.publish(pose_message(target,self.cfg['robot_frame'],self.get_clock().now().to_msg()))
        if status!=self.last_status:
            self.status_pub.publish(String(data=status));self.last_status=status


def main():
    rclpy.init();node=RelativePoseController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.enabled_pub.publish(Bool(data=False));node.destroy_node();rclpy.shutdown()
