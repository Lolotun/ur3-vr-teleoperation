"""Only a localhost fake URCap, in an isolated ROS domain. No hardware commands."""
import json
import signal
import socket
import subprocess
import sys
import threading
import time
import rclpy
from std_msgs.msg import String


def main():
    done = threading.Event();bad = threading.Event();commands=[]
    server = socket.socket();server.bind(('127.0.0.1',0));server.listen();server.settimeout(.1)
    def serve():
        while not done.is_set():
            try:connection,_=server.accept()
            except socket.timeout:continue
            with connection,connection.makefile('rb') as lines:
                for raw in lines:
                    command=raw.decode().strip();commands.append(command)
                    assert command.startswith('GET '),command
                    name=command.split()[1]
                    if bad.is_set():connection.sendall(b'ERROR bad\n');break
                    value={'POS':128,'PRE':255,'OBJ':2,'STA':3}[name]
                    connection.sendall(f'{name} {value}\n'.encode())
    thread=threading.Thread(target=serve,daemon=True);thread.start()
    rclpy.init();node=rclpy.create_node('test_measured_gripper');states=[]
    subscription=node.create_subscription(String,'/teleop/gripper_state',lambda m:states.append(json.loads(m.data)),10)
    child=subprocess.Popen([sys.executable,'-m','teleop_core.robotiq_feedback','--ros-args',
        '-p','robot_ip:=127.0.0.1','-p',f'robot_port:={server.getsockname()[1]}'])
    def wait(predicate):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise AssertionError('feedback did not arrive')
    try:
        wait(lambda:any(s.get('valid') for s in states))
        good=next(s for s in states if s.get('valid'))
        assert good['position']==128 and good['requested_position']==255
        assert good['object_status']==2 and good['contact_detected']
        assert abs(good['position_norm']-128/255)<1e-6
        bad.set()
        wait(lambda:any(not s.get('valid') for s in states))
        invalid=next(s for s in states if not s.get('valid'))
        assert 'position' not in invalid and 'error' in invalid
        print('PASS: GET-only ROS feedback, measured/requested separation, invalid reads do not reuse old position',flush=True)
    finally:
        child.send_signal(signal.SIGINT);child.wait(timeout=3)
        node.destroy_node();rclpy.try_shutdown();done.set();thread.join(timeout=1);server.close()


if __name__=='__main__':main()
