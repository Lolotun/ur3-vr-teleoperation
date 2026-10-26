import math
import unittest
from teleop_core.pose_control import (
    Pose, RelativeController, ServoSession, from_rotvec, distance, relative_target,
)


class Control:
    def __init__(self): self.sent=[];self.stops=0;self.safe=True;self.accept=True
    def isPoseWithinSafetyLimits(self,p): return self.safe
    def servoL(self,p,*args): self.sent.append(p);return self.accept
    def servoStop(self): self.stops+=1


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.robot=Pose((.4,.1,.3),from_rotvec((.2,-.3,.4)))
        self.hand=Pose((1.,2.,3.),from_rotvec((.1,.2,.3)))

    def test_initial_clutch_never_jumps_and_requires_release(self):
        core=RelativeController();self.assertIsNone(core.update(self.hand,self.robot))
        core.release();target=core.update(self.hand,self.robot)
        self.assertLess(max(distance(target,self.robot)),1e-10)

    def test_anchor_does_not_follow_actual_robot(self):
        core=RelativeController();core.release();core.update(self.hand,self.robot)
        moved=Pose((1.05,2.,3.),self.hand.q)
        a=core.update(moved,self.robot)
        b=core.update(moved,Pose((.45,.1,.3),self.robot.q))
        self.assertEqual(a,b);self.assertAlmostEqual(a.p[0],.45)

    def test_mapping_rotates_translation_and_rotation(self):
        origin=Pose((0.,0.,0.),(0.,0.,0.,1.))
        hand=Pose((.1,0.,0.),from_rotvec((.2,0.,0.)))
        target=relative_target(hand,origin,origin,from_rotvec((0.,0.,math.pi)),1.)
        self.assertAlmostEqual(target.p[0],-.1)
        self.assertLess(max(distance(target,Pose((-.1,0.,0.),from_rotvec((-.2,0.,0.))))),1e-9)

    def test_reanchor_uses_new_actual_pose(self):
        core=RelativeController();core.release();core.update(self.hand,self.robot)
        core.release();new=Pose((.3,.2,.4),from_rotvec((0.,0.,.7)))
        self.assertLess(max(distance(core.update(self.hand,new),new)),1e-10)

    def test_fault_cannot_resume_while_held(self):
        core=RelativeController();core.release();core.update(self.hand,self.robot);core.fault()
        self.assertIsNone(core.update(self.hand,self.robot))
        core.release();self.assertIsNotNone(core.update(self.hand,self.robot))

    def test_excess_displacement_latches(self):
        core=RelativeController();core.release();core.update(self.hand,self.robot)
        self.assertIsNone(core.update(Pose((2.,2.,3.),self.hand.q),self.robot))
        self.assertIsNone(core.update(self.hand,self.robot))

    def test_rotvec_roundtrip_and_quaternion_sign(self):
        for r in [(0.,0.,0.),(.2,.3,-.4),(math.pi,0.,0.)]:
            p=Pose.from_ur([.1,.2,.3,*r]);q=Pose.from_ur(p.ur())
            self.assertLess(max(distance(p,q)),1e-9)
            neg=Pose(p.p,tuple(-x for x in p.q));self.assertLess(distance(p,neg)[1],1e-9)

    def test_invalid_data(self):
        for p,q in [((float('nan'),0.,0.),(0.,0.,0.,1.)),((0.,0.,0.),(0.,0.,0.,0.))]:
            with self.assertRaises(ValueError): Pose(p,q)

    def test_preview_never_constructs_or_calls_control(self):
        session=ServoSession(None)
        self.assertEqual(session.step(self.robot,self.hand,.008),self.hand)
        self.assertIsNone(session.step(self.robot,None,.008))
        self.assertIsNone(session.command)

    def test_servo_bounds_translation_and_orientation(self):
        control=Control();session=ServoSession(control,linear_speed=.02,angular_speed=.1)
        actual=Pose((0.,0.,0.),(0.,0.,0.,1.))
        target=Pose((.1,0.,0.),from_rotvec((0.,0.,.5)))
        result=session.step(actual,target,.008)
        d,a=distance(actual,result)
        self.assertAlmostEqual(d,.02*.008);self.assertAlmostEqual(a,.1*.008)
        session.stop();session.stop();self.assertEqual(control.stops,1)

    def test_unsafe_target_not_sent(self):
        control=Control();control.safe=False;session=ServoSession(control)
        with self.assertRaises(RuntimeError): session.step(self.robot,self.hand,.008)
        self.assertFalse(control.sent)

    def test_rejected_command_gets_stop(self):
        control=Control();control.accept=False;session=ServoSession(control)
        with self.assertRaises(RuntimeError): session.step(self.robot,self.hand,.008)
        self.assertEqual(control.stops,1)


if __name__=='__main__': unittest.main()
