"""Two-hand clutch and tool pose target with optional right axial support, independent of robot model."""
import math
import uuid
import numpy as np
from .vr.mapping import robot_pose, relative_target


class AcceptedReference:
    """Accepted position/time only; rejected or discarded proposals do not advance it.

    Times use the same control clock at capture/solve sampling, not VR event time
    or service reply time. Long gaps and clock rollback require a new capture.
    """
    def __init__(self, max_interval=.1):
        self.max_interval = float(max_interval)
        if not math.isfinite(self.max_interval) or not 0 < self.max_interval <= .1:
            raise ValueError('BIMANUAL_INTEGRATION_INTERVAL_CONFIG')
        self.reset()

    def reset(self):
        self.positions = None
        self.sampled_at = None

    def capture(self, positions, sampled_at):
        self.reset()
        positions = tuple(float(v) for v in positions)
        if not positions or not all(math.isfinite(v) for v in (*positions, sampled_at)):
            raise ValueError('BIMANUAL_REFERENCE_INVALID')
        self.positions, self.sampled_at = positions, sampled_at

    def step(self, sampled_at):
        if self.positions is None:
            raise ValueError('BIMANUAL_REFERENCE_NOT_CAPTURED')
        dt = sampled_at-self.sampled_at
        if not math.isfinite(dt) or not 0 < dt <= self.max_interval:
            raise ValueError('BIMANUAL_INTEGRATION_INTERVAL')
        return self.positions, dt

    def accept(self, positions, sampled_at):
        self.step(sampled_at)
        positions = tuple(float(v) for v in positions)
        if len(positions) != len(self.positions) or not all(math.isfinite(v) for v in positions):
            raise ValueError('BIMANUAL_REFERENCE_INVALID')
        self.positions, self.sampled_at = positions, sampled_at


class BimanualSession:
    def __init__(self, config, generation, selected_at):
        self.cfg = config
        self.generation = generation
        self.selected_at = selected_at
        self.identity = uuid.uuid4().hex
        self.source = None
        self.seq = -1
        self.mode = 'idle'
        self.released = False
        self.pressed_since = None
        self.anchor_input = self.anchor_tool = None
        self.current_input = None
        self.stamp = 0.
        self.target = None
        self.clutch_seq = None
        self.grasp_mode = 'rigid'
        self.grasp_generation = 0
        self.axial = config.get('right_axial_grasp')
        self.trigger_armed = False
        if self.axial is not None:
            self.press = float(self.axial.get('press_threshold', .7))
            self.release = float(self.axial.get('release_threshold', .3))
            if not 0 <= self.release < self.press <= 1:
                raise ValueError('BIMANUAL_TRIGGER_THRESHOLDS')

    @staticmethod
    def paired_pose(left, right, left_orientation=False):
        a,b = robot_pose(left.pose),robot_pose(right.pose)
        distance = float(np.linalg.norm(a[:3,3]-b[:3,3]))
        if not .05 <= distance <= 1.2:
            raise ValueError('VR_GRASP_GEOMETRY')
        if left_orientation:
            pose = a.copy()
            pose[:3,3] = (a[:3,3]+b[:3,3])/2
            return pose
        u,s,vt = np.linalg.svd(a[:3,:3]+b[:3,:3])
        if s[-1] < .1:
            raise ValueError('VR_ORIENTATION_DEGENERATE')
        rotation=u @ vt
        if np.linalg.det(rotation) < .999:
            raise ValueError('VR_ORIENTATION_DEGENERATE')
        pose=np.eye(4)
        pose[:3,:3]=rotation
        pose[:3,3]=(a[:3,3]+b[:3,3])/2
        return pose

    def ingest(self, left, right, now):
        frames=(left,right)
        if (any(f is None or not f.valid or f.stop or f.init or not f.source_id or
                not 0 <= now-f.stamp <= self.cfg['input_max_age'] or
                f.stamp <= self.selected_at or
                not all(math.isfinite(v) and 0 <= v <= 1 for v in (f.grip,f.trigger))
                for f in frames) or left.source_id != right.source_id or
                left.seq != right.seq or abs(left.stamp-right.stamp) > .02):
            raise ValueError('VR_PAIR_INVALID')
        if self.source is not None and self.source != left.source_id:
            raise ValueError('VR_CONNECTION_CHANGED')
        if left.seq <= self.seq:
            return False
        clutch_seq = (left.clutch_seq, right.clutch_seq)
        if (any(not isinstance(value, int) or value < 0 for value in clutch_seq) or
                self.clutch_seq is not None and
                any(value < previous for value, previous in zip(clutch_seq, self.clutch_seq))):
            raise ValueError('VR_CLUTCH_UNORDERED')
        # A nonzero counter on the first frame is historical, not a new release.
        if self.clutch_seq is not None and clutch_seq != self.clutch_seq:
            # The receiver can coalesce a brief release into the next pressed
            # frame. Enter idle even when both current grip values are high;
            # the runtime then holds and discards its pending worker result.
            self.released = True
            self.pressed_since = None
            self.mode = 'idle'
            self.anchor_input = self.anchor_tool = self.target = None
        self.clutch_seq = clutch_seq
        desired = self.grasp_mode
        if self.axial is not None:
            if right.trigger < self.release:
                self.trigger_armed = True
                desired = 'rigid'
            elif right.trigger > self.press and self.trigger_armed and min(left.grip,right.grip) > .7:
                desired = 'right_axial'
        if desired != self.grasp_mode:
            self.grasp_mode = desired
            self.grasp_generation += 1
            self.reanchor()
        pose=self.paired_pose(left,right,left_orientation=self.axial is not None)
        self.current_input=pose
        self.source,self.seq,self.stamp=left.source_id,left.seq,max(left.stamp,right.stamp)
        if min(left.grip,right.grip) < .3:
            self.released=True
            self.pressed_since=None
            self.mode='idle'
            self.anchor_input=self.anchor_tool=self.target=None
        elif min(left.grip,right.grip) > .7 and self.released:
            if self.pressed_since is None:
                self.pressed_since=self.stamp
            if self.mode == 'idle' and self.stamp-self.pressed_since >= self.cfg['engage_seconds']:
                self.mode='follow'
                self.anchor_input=pose
                self.anchor_tool=self.target=None
        if self.mode == 'follow' and self.anchor_input is None:
            self.anchor_input = pose
        if self.mode == 'follow' and self.anchor_tool is not None:
            self.target=np.asarray(relative_target(self.anchor_input,pose,self.anchor_tool,
                self.cfg['position_scale'],mapping_mode='robot_base',
                operator_yaw_rad=self.cfg['operator_yaw_rad'])).reshape(4,4)
        return True

    def reanchor(self):
        """Discard pending mapping at a confirmed-stop constraint transition."""
        self.anchor_input = self.anchor_tool = self.target = None

    def capture(self, tool_pose):
        if self.mode != 'follow' or self.anchor_input is None:
            raise ValueError('BIMANUAL_NOT_ENGAGED')
        self.anchor_tool=np.asarray(tool_pose,dtype=float).reshape(4,4)
        self.target=np.asarray(relative_target(self.anchor_input,self.current_input,self.anchor_tool,
            self.cfg['position_scale'],mapping_mode='robot_base',
            operator_yaw_rad=self.cfg['operator_yaw_rad'])).reshape(4,4)
