"""Read-only ROS trace recorder and offline summary of RTDE execution."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

from teleop_core.ur3_model import UR3_ARM_JOINTS

TRACE_TOPIC = '/teleop/rtde_diagnostics'


def json_safe(value):
    """Keep JSON interoperable: a nonfinite number is explicitly null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def stats(values):
    values = sorted(x for x in values if isinstance(x, (int, float)) and math.isfinite(x))
    if not values:
        return None
    return {'median': values[len(values) // 2],
            'p95': values[max(0, math.ceil(.95 * len(values)) - 1)], 'max': values[-1]}


def summarize(path):
    topics, statuses, servo_statuses = Counter(), Counter(), Counter()
    rows, invalid_trajectories = [], 0
    with open(path) as stream:
        for line in stream:
            event = json.loads(line)
            topic = event.get('topic')
            if not topic:
                continue
            topics[topic] += 1
            if topic == TRACE_TOPIC:
                rows.append(event['data'])
            elif topic == '/teleop/servo/joint_trajectory':
                invalid_trajectories += bool(event['data'].get('nonfinite'))
    leads = [[] for _ in UR3_ARM_JOINTS]
    actual_speeds = [[] for _ in UR3_ARM_JOINTS]
    requested_speeds = [[] for _ in UR3_ARM_JOINTS]
    sent = [row for row in rows if row.get('command_sent')]
    for row in rows:
        statuses[row.get('status')] += 1
        servo_statuses[str(row.get('servo_status'))] += 1
    for row in sent:
        for i in range(6):
            a, c = row['q_actual'][i], row['q_cmd'][i]
            if a is not None and c is not None:
                leads[i].append(abs(c - a))
            for key, destination in [('qd_actual', actual_speeds), ('qd_servo', requested_speeds)]:
                value = row[key][i]
                if value is not None:
                    destination[i].append(abs(value))
    gaps = sum(max(0, b['sequence'] - a['sequence'] - 1) for a, b in zip(rows, rows[1:]))
    return {
        'topic_counts': dict(topics), 'bridge_samples': len(rows), 'sent_samples': len(sent),
        'nonfinite_trajectory_messages': invalid_trajectories,
        'bridge_status_samples': dict(statuses), 'servo_status_samples': dict(servo_statuses),
        'diagnostics_dropped_in_bridge': max((r.get('diagnostics_dropped', 0) for r in rows), default=0),
        'sequence_gaps': gaps,
        'config': rows[0].get('config') if rows else None,
        'cycle_interval_ms': stats([r['cycle_interval_s'] * 1000 for r in rows
                                    if r.get('cycle_interval_s') is not None]),
        # This includes validation and servoJ, not only wire transport time.
        'command_step_ms': stats([r['timings_s']['session_step_total'] * 1000 for r in sent]),
        'speed_scaling_during_commands': stats([r.get('getSpeedScaling') for r in sent]),
        'target_speed_fraction_during_commands': stats([r.get('getTargetSpeedFraction') for r in sent]),
        'anti_windup_limited_samples': sum(r.get('anti_windup_scale') is not None
                                          and r['anti_windup_scale'] < .999999 for r in sent),
        'anti_windup_scale_during_commands': stats([r.get('anti_windup_scale') for r in sent]),
        'joints': {name: {'abs_lead_rad': stats(leads[i]),
                          'abs_actual_velocity_rad_s': stats(actual_speeds[i]),
                          'abs_servo_velocity_rad_s': stats(requested_speeds[i])}
                   for i, name in enumerate(UR3_ARM_JOINTS)},
    }


def record(path, duration, ros_args):
    import rclpy
    from std_msgs.msg import String, Int8, Bool
    from trajectory_msgs.msg import JointTrajectory
    from geometry_msgs.msg import TwistStamped

    rclpy.init(args=ros_args)
    node = rclpy.create_node('rtde_trace_recorder')
    subscriptions = []
    count = Counter()
    try:
        with open(path, 'x') as stream:
            stream.write(json.dumps({'kind': 'metadata', 'schema': 1,
                                     'joint_names': UR3_ARM_JOINTS, 'start_wall_ns': time.time_ns()}) + '\n')

            def callback(topic):
                def accept(message):
                    if topic == TRACE_TOPIC:
                        data = json.loads(message.data)
                    elif isinstance(message, JointTrajectory):
                        data = {'names': list(message.joint_names),
                                'points': [{'q': list(p.positions), 'qd': list(p.velocities)}
                                           for p in message.points],
                                'nonfinite': any(not math.isfinite(x) for p in message.points
                                                 for x in list(p.positions) + list(p.velocities))}
                    elif isinstance(message, TwistStamped):
                        a, b = message.twist.linear, message.twist.angular
                        data = {'linear': [a.x, a.y, a.z], 'angular': [b.x, b.y, b.z]}
                    else:
                        data = message.data
                    event = {'topic': topic, 'received_ros_ns': node.get_clock().now().nanoseconds,
                             'received_monotonic_ns': time.monotonic_ns(), 'data': data}
                    stream.write(json.dumps(json_safe(event), allow_nan=False) + '\n')
                    count[topic] += 1
                return accept

            for topic, kind in [(TRACE_TOPIC, String), ('/teleop/rtde_status', String),
                                ('/teleop/servo/status', Int8), ('/teleop/tracking_status', String),
                                ('/teleop/servo/active', Bool), ('/teleop/servo/joint_trajectory', JointTrajectory),
                                ('/teleop/servo/twist', TwistStamped)]:
                subscriptions.append(node.create_subscription(kind, topic, callback(topic), 1000))
            node.create_timer(.5, stream.flush)
            print(f'Recording to {path} for {duration:g}s. No commands are published.', flush=True)
            end = time.monotonic() + duration
            try:
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.1)
            except KeyboardInterrupt:
                pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not count[TRACE_TOPIC]:
        print('No bridge diagnostics received. Enable publish_diagnostics on /ur_rtde_bridge.')
    print(json.dumps(summarize(path), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New JSONL file; existing files are never overwritten')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--summarize', type=Path, help='Summarize an existing trace without ROS')
    args, ros_args = parser.parse_known_args()
    if args.summarize:
        print(json.dumps(summarize(args.summarize), indent=2))
        return
    if args.output is None or not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--output and a positive finite --duration are required')
    record(args.output, args.duration, ros_args)


if __name__ == '__main__':
    main()
