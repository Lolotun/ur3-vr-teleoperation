"""ROS integration test with fake RTDE only. Never opens a robot connection."""
import sys
import types
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/ros2_ws/src/teleop_core'))

shared={'pose':[.4,0.,.3,0.,0.,0.],'controls':[],'freeze':False,'stamp':0.}


class Receive:
    def __init__(self,*args): pass
    def isConnected(self): return True
    def getTimestamp(self):
        if not shared['freeze']: shared['stamp']=time.monotonic()
        return shared['stamp']
    def getActualTCPPose(self): return list(shared['pose'])
    def getActualQ(self): return [0.]*6
    def getSafetyMode(self): return 1
    def getRobotMode(self): return 7
    def disconnect(self): pass


class Control:
    def __init__(self,*args): self.calls=0;self.stops=0;shared['controls'].append(self)
    def isConnected(self): return True
    def initPeriod(self): return time.monotonic()
    def waitPeriod(self,t): time.sleep(max(0.,.008-(time.monotonic()-t)))
    def isPoseWithinSafetyLimits(self,p): return True
    def servoL(self,p,*args): self.calls+=1;shared['pose']=list(p);return True
    def servoStop(self): self.stops+=1
    def stopScript(self): pass
    def disconnect(self): pass


sys.modules['rtde_receive']=types.SimpleNamespace(RTDEReceiveInterface=Receive)
sys.modules['rtde_control']=types.SimpleNamespace(RTDEControlInterface=Control)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from teleop_core.relative_pose_controller import RelativePoseController
from teleop_core.ur_rtde_bridge import UrRtdeBridge


class Integration(unittest.TestCase):
    def run_mode(self,motion):
        shared.update(pose=[.4,0.,.3,0.,0.,0.],controls=[],freeze=False,stamp=0.)
        rclpy.init(args=['--ros-args','-p','robot_ip:=mock','-p',f'enable_motion:={str(motion).lower()}'])
        bridge=UrRtdeBridge();relative=RelativePoseController();source=rclpy.create_node('fake_unity')
        executor=SingleThreadedExecutor()
        for n in [bridge,relative,source]: executor.add_node(n)
        poses=source.create_publisher(PoseStamped,'/vr/right_hand/pose',1)
        active=source.create_publisher(Bool,'/vr/right_hand/active',1)
        targets=[];source.create_subscription(PoseStamped,'/teleop/target_pose',targets.append,1)
        def pump(seconds,held=False,x=0.,send_pose=True):
            end=time.monotonic()+seconds;next_send=0.
            while time.monotonic()<end:
                if time.monotonic()>=next_send:
                    next_send=time.monotonic()+1/30
                    active.publish(Bool(data=held))
                    if send_pose:
                        msg=PoseStamped();msg.header.frame_id='vr_robot_base'
                        msg.header.stamp=source.get_clock().now().to_msg()
                        msg.pose.orientation.w=1.;msg.pose.position.x=x;poses.publish(msg)
                executor.spin_once(timeout_sec=.005)
        try:
            pump(.4);pump(.25,True);pump(.5,True,.02)
            self.assertTrue(targets)
            self.assertAlmostEqual(targets[-1].pose.position.x,.38,places=4)
            if motion:
                control=shared['controls'][0];self.assertGreater(control.calls,0)
                self.assertLess(shared['pose'][0],.4)
            else:
                self.assertFalse(shared['controls']);self.assertEqual(shared['pose'][0],.4)
            pump(.4,True,.02,False)
            if motion:
                self.assertGreater(control.stops,0);calls=control.calls
                pump(.3,True,.02)
                self.assertEqual(calls,control.calls,'Held trigger resumed after stale input')
            pump(.2,False,.02);pump(.25,True,.02)
            if motion: self.assertGreater(control.calls,calls)
            # Frozen receive values must stop feedback publication and fault the worker.
            shared['freeze']=True;pump(.35,True,.02)
            self.assertIn('feedback stopped',bridge.status)
        finally:
            executor.shutdown()
            bridge.destroy_node();relative.destroy_node();source.destroy_node();rclpy.shutdown()

    def test_preview(self): self.run_mode(False)
    def test_motion_with_fake_interfaces(self): self.run_mode(True)


if __name__=='__main__': unittest.main()
