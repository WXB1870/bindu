"""Read-only display state. Poses are robot-base transforms, never commands."""
import json
import numpy as np
from .vr.mapping import pose_matrix


class TeleopFeedback:
    def __init__(self, config):
        self.cfg = config
        self.task = ''
        self.active = False
        self.mode = 'WAITING'
        self.reason = 'Start teleop session; release grip, then hold grip'
        self.event_stamp = float('-inf')
        self.mode_stamp = float('-inf')
        self.target = None
        self.target_stamp = float('-inf')
        self.ik_code = ''

    def event(self, kind, code, task, stamp):
        if stamp < self.event_stamp:
            return
        if kind == 'TELEOP_STARTED':
            self.task, self.active = task, True
            self.mode, self.reason = 'PAUSED', 'Release grip, then hold grip to follow'
            self.target = None
            self.ik_code = ''
        elif kind == 'TELEOP_GOAL_REJECTED' and not self.active:
            self.mode, self.reason = 'REJECTED', code
            self.target = None
        elif task != self.task or not self.active:
            return
        elif kind == 'TELEOP_MODE':
            self.mode_stamp = stamp
            self.mode = 'FOLLOW' if code == 'follow' else 'PAUSED'
            self.reason = 'Waiting for target' if code == 'follow' else 'Grip released; hold grip to resume'
            self.target = None
        elif kind == 'TELEOP_IK_RESULT':
            data = json.loads(code)
            # A worker result may arrive after release/re-anchor. Never resurrect it.
            if self.mode != 'FOLLOW' or data.get('input_stamp', 0) < self.mode_stamp:
                return
            self.ik_code = data['code']
            self.target = pose_matrix(data['target']).ravel().tolist() if data['target'] else None
            self.target_stamp = data['input_stamp']
        elif kind == 'TELEOP_ENDED':
            self.active = False
            self.mode, self.reason = 'ENDED', json.loads(code)['code']
            self.target = None
        else:
            return
        self.event_stamp = stamp

    def snapshot(self, now, measured, measured_stamp, input_stamp, input_valid, feedback_error=''):
        mode, reason = self.mode, self.reason
        target = self.target
        feedback_age = now - measured_stamp
        if measured is None or not 0 <= feedback_age <= self.cfg['feedback_max_age']:
            measured = None
            if self.active:
                reason = feedback_error or 'TELEOP_FEEDBACK_STALE'
        if self.active and not input_valid:
            reason = 'VR_TRACKING_INVALID'
        elif self.active and not 0 <= now-input_stamp <= self.cfg['input_max_age']:
            reason = 'VR_INPUT_TIMEOUT'
        elif self.mode == 'FOLLOW' and measured is not None:
            reason = self.ik_code or 'Waiting for target'
        if (not self.active or self.mode != 'FOLLOW' or measured is None or not input_valid
                or not 0 <= now-input_stamp <= self.cfg['input_max_age']
                or not 0 <= now-self.target_stamp <= self.cfg['command_max_age']):
            target = None
        if self.mode == 'FOLLOW' and target is None and reason == self.ik_code:
            reason = 'Waiting for fresh target'
        error = None
        if target is not None and measured is not None:
            a, b = pose_matrix(target), pose_matrix(measured)
            error = {'position_m': float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                     'rotation_rad': float(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3])-1)/2, -1, 1)))}
        return {'mode': mode, 'reason': reason, 'task': self.task, 'side': self.cfg['side'],
                'target': target, 'measured': measured, 'error': error,
                'feedback_age_s': feedback_age if np.isfinite(feedback_age) else None,
                'simulated': True, 'frame': 'robot_base'}


def display_snapshot(snapshot, age):
    """A live receiver clears stale observer data even if the observer has died."""
    if snapshot is None or not 0 <= age <= .6:
        return {'mode': 'DISPLAY_STALE', 'reason': 'Display unavailable; check ROS observer',
                'target': None, 'measured': None, 'error': None, 'task': '', 'side': 'left'}
    return snapshot
