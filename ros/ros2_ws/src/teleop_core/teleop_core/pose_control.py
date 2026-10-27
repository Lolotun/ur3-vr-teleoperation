"""ROS-independent pose mapping and bounded Cartesian setpoints (quaternions xyzw)."""
from dataclasses import dataclass
import math
import time


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def unit(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q) or norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    n = norm(q)
    return tuple(x/n for x in q)


def mul(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return unit((w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                 w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z))


def inv(q):
    x,y,z,w = unit(q)
    return (-x,-y,-z,w)


def rotate(q, v):
    x,y,z,w = unit(q); a,b,c = v
    t = (2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a))
    return (a+w*t[0]+y*t[2]-z*t[1], b+w*t[1]+z*t[0]-x*t[2], c+w*t[2]+x*t[1]-y*t[0])


def from_rotvec(v):
    angle = norm(v)
    if angle < 1e-12: return (0.,0.,0.,1.)
    return unit(tuple(x*math.sin(angle/2)/angle for x in v)+(math.cos(angle/2),))


def rotvec(q):
    q = unit(q)
    if q[3] < 0: q = tuple(-x for x in q)
    n = norm(q[:3])
    if n < 1e-12: return (0.,0.,0.)
    angle = 2*math.atan2(n, q[3])
    return tuple(x*angle/n for x in q[:3])


@dataclass(frozen=True)
class Pose:
    p: tuple
    q: tuple

    def __post_init__(self):
        if len(self.p) != 3 or not all(math.isfinite(x) for x in self.p):
            raise ValueError('Invalid position')
        object.__setattr__(self, 'q', unit(self.q))

    @classmethod
    def from_ur(cls, values):
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            raise ValueError('Invalid UR pose')
        return cls(tuple(values[:3]), from_rotvec(values[3:]))

    def ur(self):
        return list(self.p + rotvec(self.q))


def distance(a, b):
    return norm(tuple(x-y for x,y in zip(a.p,b.p))), norm(rotvec(mul(a.q,inv(b.q))))


def relative_target(hand, hand_anchor, robot_anchor, mapping, scale):
    offset = rotate(mapping, tuple(x-y for x,y in zip(hand.p,hand_anchor.p)))
    delta = mul(mapping, mul(mul(hand.q,inv(hand_anchor.q)),inv(mapping)))
    return Pose(tuple(x+scale*d for x,d in zip(robot_anchor.p,offset)), mul(delta,robot_anchor.q))


def bounded_step(current, target, linear_step, angular_step):
    delta = tuple(x-y for x,y in zip(target.p,current.p)); length = norm(delta)
    f = min(1., linear_step/max(length,1e-12))
    rotation = rotvec(mul(target.q, inv(current.q))); angle = norm(rotation)
    g = min(1., angular_step/max(angle,1e-12))
    return Pose(tuple(x+f*d for x,d in zip(current.p,delta)),
                mul(from_rotvec(tuple(g*x for x in rotation)),current.q))


class RelativeController:
    """Release-to-rearm clutch; anchors are snapshots, never moving feedback."""
    def __init__(self, mapping=(0.,0.,0.,1.), scale=1., max_displacement=.25, max_rotation=1.):
        if scale <= 0 or max_displacement <= 0 or max_rotation <= 0:
            raise ValueError('Mapping limits must be positive')
        self.mapping=unit(mapping); self.scale=scale
        self.max_displacement=max_displacement; self.max_rotation=max_rotation
        self.released=False; self.held=False; self.anchors=None

    def release(self):
        self.released=True; self.held=False; self.anchors=None

    def fault(self):
        self.released=False; self.held=False; self.anchors=None

    def update(self, hand, actual):
        if not self.held:
            if not self.released: return None
            self.anchors=(hand,actual); self.held=True; self.released=False
        target=relative_target(hand,*self.anchors,self.mapping,self.scale)
        d,a=distance(self.anchors[1],target)
        if d>self.max_displacement or a>self.max_rotation:
            self.fault(); return None
        return target


class ServoSession:
    """One owner of control calls; preview never owns a ControlInterface."""
    def __init__(self, control, linear_speed=.02, angular_speed=.12, lookahead=.1, gain=300.,
                 pose_check_mode='robot', max_displacement=.25, max_rotation=1.):
        if pose_check_mode not in ('robot','local'):
            raise ValueError('pose_check_mode must be robot or local')
        for value in (linear_speed,angular_speed,max_displacement,max_rotation):
            if not math.isfinite(value) or value<=0:
                raise ValueError('Servo limits must be finite and positive')
        self.control=control; self.linear_speed=linear_speed; self.angular_speed=angular_speed
        self.lookahead=lookahead; self.gain=gain; self.command=None
        self.pose_check_mode=pose_check_mode
        self.max_displacement=max_displacement;self.max_rotation=max_rotation
        self.anchor=None
        self.timings={}

    def stop(self):
        self.anchor=None
        if self.command is not None:
            self.command=None
            if self.control is not None: self.control.servoStop()

    def step(self, actual, target, dt):
        self.timings={}
        if target is None:
            self.stop(); return None
        if self.control is None: return target
        if not math.isfinite(dt) or dt<=0:
            self.stop();raise RuntimeError('Invalid servo period')
        current=self.command or actual
        command=bounded_step(current,target,self.linear_speed*dt,self.angular_speed*dt)
        values=command.ur()
        started=time.monotonic()
        if self.pose_check_mode=='robot':
            safe=self.control.isPoseWithinSafetyLimits(values)
            self.timings['isPoseWithinSafetyLimits']=time.monotonic()-started
        else:
            # Cartesian guard only: no IK, collision model or UR safety-plane checks.
            # Reconstruct poses to reject malformed/non-finite values at this boundary.
            try:
                actual=Pose(actual.p,actual.q);target=Pose(target.p,target.q)
                command=Pose(command.p,command.q)
                anchor=self.anchor or actual
                displacement,rotation=distance(anchor,target)
                step,turn=distance(current,command)
                safe=(displacement<=self.max_displacement and rotation<=self.max_rotation
                      and step<=self.linear_speed*dt+1e-10
                      and turn<=self.angular_speed*dt+1e-10)
            except ValueError:
                safe=False
            self.timings['local_pose_check']=time.monotonic()-started
        if not safe:
            self.stop(); raise RuntimeError(f'Target rejected by {self.pose_check_mode} pose check')
        # Mark before the call so shutdown also stops a command whose acknowledgement fails.
        self.command=command
        if self.anchor is None: self.anchor=actual
        started=time.monotonic()
        accepted=self.control.servoL(values,0.,0.,dt,self.lookahead,self.gain)
        self.timings['servoL']=time.monotonic()-started
        if not accepted:
            self.stop(); raise RuntimeError('servoL rejected')
        return command
