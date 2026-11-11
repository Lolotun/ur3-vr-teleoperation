"""Read-only URCap feedback. This module never sends SET, activation or motion commands."""
import json
import math
import socket
import threading
import time


def parse_register(variable, response):
    try:
        fields = response.decode('ascii').split()
        if len(fields) != 2 or fields[0] != variable:
            raise ValueError('unexpected register response')
        value = int(fields[1])
    except (UnicodeError, ValueError) as exc:
        raise ValueError(f'invalid GET {variable} response: {response!r}') from exc
    maximum = 255 if variable in ('POS', 'PRE') else 3
    if not 0 <= value <= maximum:
        raise ValueError(f'GET {variable} outside 0..{maximum}: {value}')
    return value


def read_feedback(host, port=63352, timeout=.2, clock_ns=time.time_ns,
                  open_position=0, close_position=255):
    """One bounded polling cycle on a separate read-only connection, like the ur_rtde URCap example."""
    if not math.isfinite(timeout) or timeout <= 0 or open_position == close_position:
        raise ValueError('invalid feedback timeout or position endpoints')
    deadline = time.monotonic()+timeout
    values = {}
    with socket.create_connection((host, port), timeout=timeout) as sock:
        for register in ('POS', 'PRE', 'OBJ', 'STA'):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('gripper feedback cycle exceeded deadline')
            sock.settimeout(remaining)
            sock.sendall(f'GET {register}\n'.encode('ascii'))
            response = sock.recv(128)
            if not response:
                raise ConnectionError('gripper disconnected during GET')
            values[register] = parse_register(register, response)
            if register == 'POS':
                stamp_ns = clock_ns()
    normalize = lambda v: max(0., min(1., (v-open_position)/(close_position-open_position)))
    return {'schema': 1, 'valid': values['STA'] == 3, 'stamp_ns': stamp_ns,
            'position': values['POS'], 'position_norm': normalize(values['POS']),
            'requested_position': values['PRE'], 'requested_norm': normalize(values['PRE']),
            'object_status': values['OBJ'], 'activation_status': values['STA'],
            'moving': values['OBJ'] == 0, 'contact_detected': values['OBJ'] in (1, 2),
            'at_destination': values['OBJ'] == 3,
            'sample_span_s': max(0., timeout-(deadline-time.monotonic()))}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    class FeedbackNode(Node):
        def __init__(self):
            super().__init__('robotiq_feedback')
            for k, v in {'robot_ip': '', 'robot_port': 63352, 'frequency': 10.,
                         'timeout': .2, 'open_position': 0, 'close_position': 255}.items():
                self.declare_parameter(k, v)
            cfg = {k: self.get_parameter(k).value for k in
                   ('robot_ip', 'robot_port', 'frequency', 'timeout', 'open_position', 'close_position')}
            if (not cfg['robot_ip'] or not math.isfinite(cfg['frequency']) or cfg['frequency'] <= 0
                    or not math.isfinite(cfg['timeout']) or cfg['timeout'] <= 0
                    or cfg['open_position'] == cfg['close_position']):
                raise ValueError('invalid gripper feedback configuration')
            self.done, self.lock = threading.Event(), threading.Lock()
            self.latest, self.sequence, self.published = None, 0, 0
            self.pub = self.create_publisher(String, '/teleop/gripper_state', 10)
            self.create_timer(1/30, self.publish)

            def worker():
                while not self.done.is_set():
                    start = time.monotonic()
                    try:
                        data = read_feedback(cfg['robot_ip'], cfg['robot_port'], cfg['timeout'],
                            lambda: self.get_clock().now().nanoseconds,
                            cfg['open_position'], cfg['close_position'])
                    except (OSError, ValueError) as exc:
                        data = {'schema': 1, 'valid': False, 'stamp_ns': self.get_clock().now().nanoseconds,
                                'error': str(exc)}
                    with self.lock:
                        self.latest = data
                        self.sequence += 1
                    self.done.wait(max(0., 1/cfg['frequency']-(time.monotonic()-start)))
            self.worker = threading.Thread(target=worker, name='gripper-get-only', daemon=True)
            self.worker.start()

        def publish(self):
            with self.lock:
                data, sequence = self.latest, self.sequence
            if data is not None and sequence != self.published:
                self.pub.publish(String(data=json.dumps(data, allow_nan=False)))
                self.published = sequence

        def destroy_node(self):
            self.done.set()
            self.worker.join(timeout=1.)
            return super().destroy_node()

    rclpy.init()
    node = FeedbackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
