"""Exclusive source ownership and expiring policy leases, without ROS dependencies."""
import math
from teleop_core.pose_control import Pose, distance, bounded_step


class ControlMux:
    def __init__(self, timeout=.25, feedback_timeout=.25, max_distance=.20, max_rotation=.5,
                 target_distance=.03, target_rotation=math.radians(10),
                 workspace_min=(-.6, -.6, -.01), workspace_max=(.6, .6, 1.)):
        if any(not math.isfinite(v) or v <= 0 for v in
               (timeout, feedback_timeout, max_distance, max_rotation, target_distance, target_rotation)):
            raise ValueError('policy limits and timeouts must be finite and positive')
        if target_distance > max_distance or target_rotation > max_rotation:
            raise ValueError('policy target limits must not exceed rejection limits')
        self.timeout, self.feedback_timeout = timeout, feedback_timeout
        self.max_distance, self.max_rotation = max_distance, max_rotation
        self.target_distance, self.target_rotation = target_distance, target_rotation
        self.workspace_min, self.workspace_max = workspace_min, workspace_max
        self.mode, self.epoch, self.reason = 'STOP', 0, 'STARTUP'
        self.actual = None
        self.actual_time = -math.inf
        self.switch('STOP', 0.)

    def switch(self, mode, now):
        if mode not in ('STOP', 'VR', 'POLICY'):
            raise ValueError('mode must be STOP, VR or POLICY')
        self.mode, self.epoch = mode, self.epoch + 1
        self.reason = 'SELECTED_' + mode
        self.release_until = now + .1
        self.vr_target = self.policy_target = None
        self.policy_diagnostic = None
        self.vr_target_time = self.vr_active_time = self.raw_time = self.policy_time = -math.inf
        self.vr_active = self.raw_active = self.release_seen = False
        self.policy_owner, self.policy_sequence, self.policy_started = None, 0, False
        self.grip = None

    def stop(self, reason, now):
        self.switch('STOP', now)
        self.reason = reason

    def feedback(self, pose, now):
        self.actual, self.actual_time = pose, now

    def raw_clutch(self, active, now):
        self.raw_active, self.raw_time = bool(active), now
        if not active:
            self.release_seen = True

    def vr_enable(self, active, now):
        self.vr_active, self.vr_active_time = bool(active), now
        if not active:
            self.vr_target = None

    def vr_pose(self, pose, now):
        self.vr_target, self.vr_target_time = pose, now

    def vr_grip(self, g):
        if self.mode == 'VR':
            self.grip = g

    def policy(self, event, now):
        if self.mode != 'POLICY':
            raise ValueError('POLICY mode has not been selected')
        if event.get('epoch') != self.epoch:
            raise ValueError('obsolete control epoch')
        session, sequence = event.get('session'), event.get('sequence')
        if not isinstance(session, str) or not session or type(sequence) is not int or sequence <= 0:
            raise ValueError('invalid policy owner/sequence')
        if self.policy_owner is not None and session != self.policy_owner:
            raise ValueError('another policy owns this mode epoch')
        if sequence <= self.policy_sequence:
            raise ValueError('duplicate/out-of-order policy command')
        pose = Pose(tuple(event['position']), tuple(event['quaternion']))
        g = float(event['gripper'])
        if not math.isfinite(g) or not 0 <= g <= 1:
            raise ValueError('invalid policy gripper')
        if self.actual is None or now - self.actual_time > self.feedback_timeout:
            raise ValueError('stale actual TCP')
        if any(not lo <= v <= hi for v, lo, hi in zip(pose.p, self.workspace_min, self.workspace_max)):
            raise ValueError('policy target outside workspace')
        translation, angle = distance(self.actual, pose)
        if translation > self.max_distance or angle > self.max_rotation:
            raise ValueError(
                f'policy target too far from actual TCP: distance={translation*100:.2f}cm '
                f'(limit={self.max_distance*100:.2f}cm), angle={math.degrees(angle):.2f}deg '
                f'(limit={math.degrees(self.max_rotation):.2f}deg); '
                f'actual_xyz={self.actual.p}, target_xyz={pose.p}')
        limited = bounded_step(self.actual, pose, self.target_distance, self.target_rotation)
        if any(not lo <= v <= hi for v, lo, hi in zip(limited.p, self.workspace_min, self.workspace_max)):
            raise ValueError('limited policy target outside workspace')
        self.policy_owner, self.policy_sequence = session, sequence
        self.policy_diagnostic = {
            'actual': {'position': self.actual.p, 'quaternion': self.actual.q},
            'raw': {'position': pose.p, 'quaternion': pose.q, 'gripper': g},
            'limited': {'position': limited.p, 'quaternion': limited.q, 'gripper': g},
            'distance_m': translation, 'angle_rad': angle,
            'clipped': translation > self.target_distance or angle > self.target_rotation}
        self.policy_target, self.policy_time, self.policy_started = limited, now, True
        self.grip = g

    def policy_limits(self):
        return dict(target_distance=self.target_distance, target_rotation=self.target_rotation,
                    max_distance=self.max_distance, max_rotation=self.max_rotation,
                    workspace_min=self.workspace_min, workspace_max=self.workspace_max)

    def output(self, now):
        if self.mode == 'STOP' or now < self.release_until:
            return None, None
        if self.actual is None or now - self.actual_time > self.feedback_timeout:
            if self.mode == 'POLICY' and self.policy_started:
                self.stop('STALE_FEEDBACK', now)
            return None, None
        if self.mode == 'POLICY':
            if not self.policy_started:
                return None, None
            if now - self.policy_time > self.timeout:
                self.stop('POLICY_TIMEOUT', now)
                return None, None
            pose = self.policy_target
        else:
            fresh = (self.release_seen and self.raw_active and self.vr_active
                     and now - self.raw_time <= self.timeout
                     and now - self.vr_active_time <= self.timeout
                     and self.vr_target is not None and now - self.vr_target_time <= self.timeout)
            pose = self.vr_target if fresh else None
            if now - self.raw_time > self.timeout:
                self.release_seen = False
        grip, self.grip = self.grip, None
        return pose, grip
