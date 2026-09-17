"""Single-arm clutch and relative-target state machine, independent of ROS/IK."""
import math
import uuid
import numpy as np
from bindu_contracts.teleoperation import IKRequest
from .vr.mapping import robot_pose, relative_target


class TeleopSession:
    def __init__(self, config, names):
        self.cfg, self.names = config, tuple(names)
        self.identity = uuid.uuid4().hex
        self.source = None
        self.seq = -1
        self.generation = 0
        self.mode = 'idle'
        self.released = False
        self.pressed_since = None
        self.anchor_input = self.anchor_robot = None
        self.anchor_pending = False
        self.last_requested = -1
        self.last_result = -float('inf')
        self.frame = None
        self.clutch_seq = 0

    def ingest(self, frame, now):
        if (frame.side != self.cfg['side'] or not frame.source_id or
                not isinstance(frame.seq, int) or frame.seq < 0 or
                not math.isfinite(frame.stamp) or not 0 <= now-frame.stamp <= self.cfg['input_max_age']):
            raise ValueError('VR_STALE_OR_INVALID_FRAME')
        if self.source is not None and frame.source_id != self.source:
            raise ValueError('VR_CONNECTION_CHANGED')
        if frame.seq <= self.seq:
            return False
        if not frame.valid:
            raise ValueError('VR_TRACKING_INVALID')
        robot_pose(frame.pose)
        if not all(math.isfinite(x) and 0 <= x <= 1 for x in (frame.grip, frame.trigger)):
            raise ValueError('VR_INVALID_BUTTON')
        if frame.clutch_seq < self.clutch_seq:
            raise ValueError('VR_CLUTCH_UNORDERED')
        if frame.clutch_seq != self.clutch_seq:
            self.released, self.pressed_since = True, None
            self.generation += 1
            self.mode = 'idle'
            self.anchor_input = self.anchor_robot = None
            self.anchor_pending = False
        self.clutch_seq = frame.clutch_seq
        self.source, self.seq, self.frame = frame.source_id, frame.seq, frame
        if frame.stop:
            raise ValueError('VR_STOP')
        if frame.init:
            # No verified initial posture exists yet. Never silently move to zero.
            raise ValueError('VR_INIT_NOT_CONFIGURED')
        if frame.grip < .3:
            self.released, self.pressed_since = True, None
            if self.mode != 'idle':
                self.generation += 1
                self.mode = 'idle'
                self.anchor_input = self.anchor_robot = None
                self.anchor_pending = False
        elif frame.grip > .7 and self.released:
            if self.pressed_since is None:
                self.pressed_since = frame.stamp
            if frame.stamp-self.pressed_since >= self.cfg['engage_seconds'] and self.mode == 'idle':
                self.mode = 'follow'
                self.generation += 1
        return True

    def check(self, now):
        if self.frame is None or not 0 <= now-self.frame.stamp <= self.cfg['input_max_age']:
            raise ValueError('VR_INPUT_TIMEOUT')

    def request(self, positions, feedback_stamp, now):
        self.check(now)
        if not 0 <= now-feedback_stamp <= self.cfg['feedback_max_age']:
            raise ValueError('TELEOP_FEEDBACK_STALE')
        if self.mode != 'follow' or self.anchor_pending or self.seq == self.last_requested:
            return None
        seed = tuple(positions[n] for n in self.names)
        if not all(math.isfinite(q) for q in seed):
            raise ValueError('TELEOP_INVALID_FEEDBACK')
        current = tuple(robot_pose(self.frame.pose).ravel())
        target = ()
        if self.anchor_robot is None:
            self.anchor_input = current
            self.anchor_pending = True
        else:
            target = relative_target(self.anchor_input, current, self.anchor_robot, self.cfg['position_scale'])
        self.last_requested = self.seq
        return IKRequest(self.identity+'_'+str(self.seq), self.generation, self.frame.stamp,
                         self.frame.stamp+self.cfg['command_max_age'], self.names, seed, target)

    def accept_result(self, result, now):
        req = result.request
        if (req.generation != self.generation or not req.request_id.startswith(self.identity+'_') or
                self.mode != 'follow'):
            return False
        if not 0 <= now-req.stamp < self.cfg['command_max_age'] or req.stamp <= self.last_result:
            raise ValueError('TELEOP_IK_RESULT_EXPIRED')
        if not result.success:
            raise ValueError(result.code)
        if (req.names != self.names or len(result.positions) != len(self.names) or
                not all(math.isfinite(v) for v in result.positions)):
            raise ValueError('TELEOP_IK_LAYOUT')
        if not req.target:
            self.anchor_robot = tuple(np.asarray(result.pose).reshape(16))
            self.anchor_pending = False
        self.last_result = req.stamp
        return True
