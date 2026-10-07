"""Two-hand clutch and one rigid-tool pose target, independent of robot model."""
import math
import uuid
import numpy as np
from .vr.mapping import robot_pose, relative_target


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

    @staticmethod
    def paired_pose(left, right):
        a,b = robot_pose(left.pose),robot_pose(right.pose)
        distance = float(np.linalg.norm(a[:3,3]-b[:3,3]))
        if not .05 <= distance <= 1.2:
            raise ValueError('VR_GRASP_GEOMETRY')
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
                f.stamp <= self.selected_at or not math.isfinite(f.grip)
                for f in frames) or left.source_id != right.source_id or
                left.seq != right.seq or abs(left.stamp-right.stamp) > .02):
            raise ValueError('VR_PAIR_INVALID')
        if self.source is not None and self.source != left.source_id:
            raise ValueError('VR_CONNECTION_CHANGED')
        if left.seq <= self.seq:
            return False
        pose=self.paired_pose(left,right)
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
        if self.mode == 'follow' and self.anchor_tool is not None:
            self.target=np.asarray(relative_target(self.anchor_input,pose,self.anchor_tool,
                self.cfg['position_scale'],mapping_mode='robot_base',
                operator_yaw_rad=self.cfg['operator_yaw_rad'])).reshape(4,4)
        return True

    def capture(self, tool_pose):
        if self.mode != 'follow' or self.anchor_input is None:
            raise ValueError('BIMANUAL_NOT_ENGAGED')
        self.anchor_tool=np.asarray(tool_pose,dtype=float).reshape(4,4)
        self.target=np.asarray(relative_target(self.anchor_input,self.current_input,self.anchor_tool,
            self.cfg['position_scale'],mapping_mode='robot_base',
            operator_yaw_rad=self.cfg['operator_yaw_rad'])).reshape(4,4)
