#!/usr/bin/env python3
"""Compare check+servoL with repeated servoL at a fixed, validated actual pose.

This connects a ControlInterface and activates servo control. Run with no other
robot command source. It does not consume VR targets or command an offset.
"""
import argparse
import math
import os
import statistics
import time


def run_phase(control, receive, frequency, samples, check_each):
    dt = 1.0 / frequency
    rows = []
    armed = False
    try:
        if receive.getSafetyMode() != 1 or receive.getRobotMode() != 7:
            raise RuntimeError('Robot must be in normal safety mode and running robot mode')
        speed = receive.getActualTCPSpeed()
        if any(abs(v) > 0.001 for v in speed):
            raise RuntimeError('TCP is moving; refusing to capture a holding target')
        target = list(receive.getActualTCPPose())
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise RuntimeError('Invalid actual TCP pose')
        if not control.isPoseWithinSafetyLimits(target):
            raise RuntimeError('Holding target rejected by safety check')
        previous_stamp = receive.getTimestamp()
        last_fresh = time.monotonic()
        deadline = last_fresh + 3.0
        for _ in range(samples):
            started = time.monotonic()
            if started >= deadline:
                break
            period = control.initPeriod()
            if not receive.isConnected() or not control.isConnected():
                raise RuntimeError('RTDE disconnected')
            if receive.getSafetyMode() != 1 or receive.getRobotMode() != 7:
                raise RuntimeError('Robot is no longer ready')
            stamp = receive.getTimestamp()
            if not math.isfinite(stamp):
                raise RuntimeError('Invalid feedback timestamp')
            if stamp != previous_stamp:
                previous_stamp = stamp
                last_fresh = time.monotonic()
            if time.monotonic() - last_fresh > 0.25:
                raise RuntimeError('Feedback stopped updating')
            actual = receive.getActualTCPPose()
            if len(actual) != 6 or not all(math.isfinite(v) for v in actual):
                raise RuntimeError('Invalid actual TCP pose')
            # Conservative component checks: abort rather than continue if the
            # fixed holding pose drifts. Rotation-vector wrapping can also abort.
            if math.dist(actual[:3], target[:3]) > 0.005 or math.dist(actual[3:], target[3:]) > 0.05:
                raise RuntimeError('TCP moved away from holding target')
            before_check = time.monotonic()
            if check_each and not control.isPoseWithinSafetyLimits(target):
                raise RuntimeError('Holding target rejected')
            before_servo = time.monotonic()
            armed = True
            if not control.servoL(target, 0., 0., dt, 0.1, 300.):
                raise RuntimeError('servoL rejected')
            after_servo = time.monotonic()
            control.waitPeriod(period)
            rows.append((before_servo-before_check, after_servo-before_servo,
                         time.monotonic()-started))
        return rows
    finally:
        if armed:
            control.servoStop()


def report(label, rows):
    print(f'\n{label}: {len(rows)} samples', flush=True)
    for column, name in enumerate(('safety_check', 'servoL', 'cycle_total')):
        values = [row[column]*1000 for row in rows]
        if values:
            print(f'  {name}: first={values[0]:.2f}ms '
                  f'median={statistics.median(values):.2f}ms max={max(values):.2f}ms', flush=True)
    # The first servo call can still wait for the initial safety query to clear.
    if len(rows) > 1:
        print(f'  servoL excluding first: median='
              f'{statistics.median(row[1]*1000 for row in rows[1:]):.2f}ms', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default=os.environ.get('ROBOT_IP'))
    parser.add_argument('--frequency', type=float, default=125.)
    parser.add_argument('--samples', type=int, default=50)
    parser.add_argument('--enable-control', action='store_true',
                        help='Upload control script and issue fixed-pose servo commands')
    args = parser.parse_args()
    if not args.robot_ip or not args.enable_control:
        parser.error('--robot-ip (or ROBOT_IP) and --enable-control are required')
    if not math.isfinite(args.frequency) or not 1 <= args.frequency <= 125 or not 1 <= args.samples <= 100:
        parser.error('frequency must be 1..125Hz; samples must be 1..100')
    from rtde_receive import RTDEReceiveInterface
    from rtde_control import RTDEControlInterface
    control = receive = None
    try:
        receive = RTDEReceiveInterface(args.robot_ip, args.frequency)
        control = RTDEControlInterface(args.robot_ip, args.frequency)
        for check_each, label in ((True, 'A: check before every servoL'),
                                  (False, 'B: fixed target checked once')):
            print(f'Starting {label}', flush=True)
            rows = run_phase(control, receive, args.frequency, args.samples, check_each)
            report(label, rows)
    finally:
        try:
            if control is not None:
                try:
                    control.stopScript()
                finally:
                    control.disconnect()
        finally:
            if receive is not None:
                receive.disconnect()


if __name__ == '__main__':
    main()
