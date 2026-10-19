"""Manual teleoperation mode fencing; no ROS or robot-specific resource names."""
import math
from .bimanual_session import BimanualSession, ToolPoseSession
from .vr.mapping import pose_matrix


class ModeSelection:
    def __init__(self, profile, initial_side, now, input_age=.25, feedback_age=.2, left_orientation=False, tool_config=None,
                 require_recording=True, enabled_modes=('single_arm','bimanual_tool')):
        if initial_side not in ('left','right'):
            raise ValueError('INVALID_SIDE')
        self.profile = profile
        self.left_orientation = left_orientation
        self.tool_config = tool_config
        if type(require_recording) is not bool or not enabled_modes or set(enabled_modes)-{'single_arm','bimanual_tool','independent_arms'}:
            raise ValueError('INVALID_TELEOP_REQUIREMENTS')
        self.require_recording = require_recording
        self.enabled_modes = tuple(enabled_modes)
        self.mode, self.side, self.phase = 'single_arm', initial_side, 'READY'
        self.generation, self.changed_at = 1, now
        self.target = None
        self.code = 'OK'
        if 'single_arm' not in self.enabled_modes:
            self.mode,self.side,self.phase,self.code = '','','SAFE','MODE_NOT_SELECTED'
        self.input_age, self.feedback_age = input_age, feedback_age

    def begin(self, mode, side, generation, now):
        if self.phase == 'SWITCHING':
            raise ValueError('MODE_SWITCH_BUSY')
        if generation != self.generation:
            raise ValueError('STALE_MODE_GENERATION')
        if mode not in self.enabled_modes or (mode == 'single_arm' and side not in ('left','right')) or (mode != 'single_arm' and side):
            raise ValueError('INVALID_MODE_SELECTION')
        if self.phase == 'READY' and (mode,side) == (self.mode,self.side):
            return False
        self.target = (mode,side)
        self.phase, self.code, self.changed_at = 'SWITCHING', 'STOPPING_PREVIOUS_MODE', now
        return True

    def ready(self, state, health, frames, now, resources, tool_frame=None):
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
        if self.require_recording and (health is None or not health.ready or not 0 <= now-health.stamp < .75):
            return 'RECORDER_UNAVAILABLE'
        mode, side = self.target
        if mode == 'bimanual_tool' and self.tool_config is not None:
            # A mapped tool source has no VR clutch. Manual selection still
            # requires stopped execution and fresh input after the switch.
            try:
                ToolPoseSession(self.tool_config, self.generation, self.changed_at).ingest(tool_frame, now)
            except (ValueError, TypeError) as exc:
                return str(exc) if isinstance(exc, ValueError) else 'TOOL_INPUT_INVALID'
            return ''
        sides = ('left','right') if mode != 'single_arm' else (side,)
        selected = []
        for key in sides:
            frame = frames.get(key)
            if (frame is None or not frame.valid or frame.stop or frame.init or
                    not frame.source_id or frame.side != key or
                    type(frame.seq) is not int or frame.seq < 0 or
                    type(frame.clutch_seq) is not int or frame.clutch_seq < 0 or
                    not all(math.isfinite(v) and 0 <= v <= 1 for v in (frame.grip, frame.trigger)) or
                    frame.stamp <= self.changed_at or not 0 <= now-frame.stamp <= self.input_age or
                    frame.grip >= .3):
                return 'VR_RELEASE_REQUIRED'
            selected.append(frame)
        if len(selected) == 2:
            if (selected[0].source_id != selected[1].source_id or
                    selected[0].seq != selected[1].seq or
                    abs(selected[0].stamp-selected[1].stamp) > .02):
                return 'VR_PAIR_MISMATCH'
            try:
                if mode=='independent_arms':
                    for frame in selected:pose_matrix(frame.pose)
                else:BimanualSession.paired_pose(*selected,left_orientation=self.left_orientation)
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
