"""Manual teleoperation mode fencing; no ROS or robot-specific resource names."""
import math
from .bimanual_session import BimanualSession
from .vr.mapping import pose_matrix


class ModeSelection:
    def __init__(self, profile, initial_side, now, input_age=.25, feedback_age=.2, left_orientation=False):
        if initial_side not in ('left','right'):
            raise ValueError('INVALID_SIDE')
        self.profile = profile
        self.left_orientation = left_orientation
        self.mode, self.side, self.phase = 'single_arm', initial_side, 'READY'
        self.generation, self.changed_at = 1, now
        self.target = None
        self.code = 'OK'
        self.input_age, self.feedback_age = input_age, feedback_age

    def begin(self, mode, side, generation, now):
        if self.phase == 'SWITCHING':
            raise ValueError('MODE_SWITCH_BUSY')
        if generation != self.generation:
            raise ValueError('STALE_MODE_GENERATION')
        if mode not in ('single_arm','bimanual_tool') or (mode == 'single_arm' and side not in ('left','right')) or (mode == 'bimanual_tool' and side):
            raise ValueError('INVALID_MODE_SELECTION')
        if self.phase == 'READY' and (mode,side) == (self.mode,self.side):
            return False
        self.target = (mode,side)
        self.phase, self.code, self.changed_at = 'SWITCHING', 'STOPPING_PREVIOUS_MODE', now
        return True

    def ready(self, state, health, frames, now, resources):
        if self.phase != 'SWITCHING' or state is None:
            return 'EXECUTION_NOT_READY'
        if (state.profile_hash != self.profile.digest or state.lease_active or
                state.state in ('RUNNING','STOPPING') or state.stop_failures):
            return 'EXECUTION_NOT_QUIESCENT'
        if not 0 <= now-state.feedback_stamp <= self.feedback_age or state.feedback_stamp < self.changed_at:
            return 'TELEOP_FEEDBACK_STALE'
        velocities = dict(zip(state.joints.name,state.joints.velocity))
        if any(name not in velocities or not math.isfinite(velocities[name]) or abs(velocities[name]) > .01
               for group in resources for name in self.profile.groups[group]):
            return 'TELEOP_NOT_STOPPED'
        if health is None or not health.ready or not 0 <= now-health.stamp < .75:
            return 'RECORDER_UNAVAILABLE'
        mode, side = self.target
        sides = ('left','right') if mode == 'bimanual_tool' else (side,)
        selected = []
        for key in sides:
            frame = frames.get(key)
            if (frame is None or not frame.valid or frame.stop or frame.init or
                    not frame.source_id or not math.isfinite(frame.grip) or
                    frame.stamp < self.changed_at or not 0 <= now-frame.stamp <= self.input_age or
                    frame.grip >= .3):
                return 'VR_RELEASE_REQUIRED'
            selected.append(frame)
        if len(selected) == 2:
            if (selected[0].source_id != selected[1].source_id or
                    selected[0].seq != selected[1].seq or
                    abs(selected[0].stamp-selected[1].stamp) > .02):
                return 'VR_PAIR_MISMATCH'
            try:
                BimanualSession.paired_pose(*selected,left_orientation=self.left_orientation)
            except ValueError as exc:
                if str(exc) in ('VR_GRASP_GEOMETRY', 'VR_ORIENTATION_DEGENERATE'):
                    return str(exc)
                return 'VR_TRACKING_INVALID'
        else:
            try:
                pose_matrix(selected[0].pose)
            except ValueError:
                return 'VR_TRACKING_INVALID'
        return ''

    def finish(self, now):
        if self.phase != 'SWITCHING' or self.target is None:
            raise ValueError('NO_MODE_SWITCH')
        self.mode,self.side = self.target
        self.target = None
        self.generation += 1
        self.changed_at, self.phase, self.code = now, 'READY', 'OK'

    def fail(self, reason, now):
        self.mode, self.side, self.target = '', '', None
        self.generation += 1
        self.changed_at, self.phase, self.code = now, 'SAFE', reason
