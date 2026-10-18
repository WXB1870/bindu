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
    if snapshot.get('pico'):
        view=dict(snapshot)
        maximum=view.get('input_max_age_s',.25)
        view['input_fresh']=bool(view.get('input_fresh') and
            view.get('input_age_s') is not None and 0<=view['input_age_s']+age<=maximum)
        if not view['input_fresh']:
            view['raw_input_valid']=False
            view['tracking_valid']={'left':False,'right':False}
        if view.get('feedback_age_s') is None or not 0<=view['feedback_age_s']+age<=maximum:
            view['stop_confirmed']=False
        return view
    return snapshot


class ToolFeedback(TeleopFeedback):
    """Tool observer distinguishes software grasp constraints from physical jaws."""
    def __init__(self, config):
        super().__init__({**config, 'side': 'tool'})
        self.reason = 'TOOL_SESSION_INACTIVE'
        self.grasp_requested = self.grasp_applied = None

    def event(self, kind, code, task, stamp):
        if stamp < self.event_stamp:
            return
        if kind == 'BIMANUAL_STARTED':
            self.task, self.active = task, True
            self.mode, self.reason = 'CAPTURING', 'WAITING_FOR_GRASP'
            self.target = None
            self.grasp_requested = 'rigid'
            self.grasp_applied = None
        elif kind == 'BIMANUAL_GOAL_REJECTED' and not self.active:
            self.mode, self.reason = 'REJECTED', code
        elif not self.active or task not in ('', self.task):
            return
        elif kind in ('BIMANUAL_PAUSE_REQUESTED', 'BIMANUAL_PAUSED',
                      'BIMANUAL_RESUME_REQUESTED', 'BIMANUAL_RESUMED'):
            self.mode = {'BIMANUAL_PAUSE_REQUESTED': 'PAUSING', 'BIMANUAL_PAUSED': 'PAUSED',
                         'BIMANUAL_RESUME_REQUESTED': 'WAITING_INPUT', 'BIMANUAL_RESUMED': 'CAPTURING'}[kind]
            self.reason, self.target = code, None
        elif kind == 'BIMANUAL_GRASP_REQUESTED':
            self.grasp_requested = json.loads(code)['mode']
            self.grasp_applied = None
            self.mode, self.reason, self.target = 'CAPTURING', 'GRASP_SWITCH', None
        elif kind == 'BIMANUAL_CAPTURED':
            self.grasp_applied = json.loads(code)['grasp_mode']
            self.grasp_requested = self.grasp_applied
            self.mode, self.reason = 'FOLLOW', ''
        elif kind == 'BIMANUAL_CLUTCH':
            self.mode = 'PAUSED' if code == 'idle' else 'CAPTURING'
            self.reason, self.target = 'CLUTCH_'+code.upper(), None
        elif kind == 'BIMANUAL_QP_RESULT':
            data = json.loads(code)
            if self.mode != 'FOLLOW':
                return
            self.target, self.target_stamp = data.get('target'), data['input_stamp']
        elif kind == 'BIMANUAL_ENDED':
            self.active = False
            self.mode, self.reason = 'ENDED', json.loads(code)['code']
            self.target, self.grasp_applied = None, None
        else:
            return
        self.event_stamp = stamp

    def snapshot(self, now, measured, measured_stamp, input_stamp, input_valid, feedback_error=''):
        # An explicit pause intentionally allows the source to stop streaming.
        paused = self.mode in ('PAUSED', 'PAUSING', 'WAITING_INPUT')
        view = super().snapshot(now, measured, measured_stamp,
                               now if paused else input_stamp, True if paused else input_valid,
                               feedback_error)
        if self.active and view['measured'] is None:
            view['reason'] = feedback_error or 'BIMANUAL_FEEDBACK_STALE'
        if self.mode != 'FOLLOW' and view['measured'] is not None:
            view['reason'] = self.reason
        view['reason'] = {'VR_TRACKING_INVALID': 'TOOL_INPUT_INVALID',
                          'VR_INPUT_TIMEOUT': 'TOOL_INPUT_TIMEOUT'}.get(view['reason'],view['reason'])
        view.update(frame=self.cfg.get('input_frame', 'robot_base'),
                    grasp_requested=self.grasp_requested, grasp_applied=self.grasp_applied,
                    physical_gripper_confirmed=None)
        return view
