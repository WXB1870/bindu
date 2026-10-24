"""Two independent relative hand targets, sharing one fenced execution stream."""
import numpy as np
import uuid
import math
from .session import TeleopSession
from .vr.mapping import relative_target, pose_matrix, engagement_yaw


class IndependentArmsSession:
    def __init__(self, config, generation, selected_at, **_):
        self.cfg=config;self.generation=generation;self.selected_at=selected_at
        self.identity=uuid.uuid4().hex
        self.hands=[TeleopSession(dict(config,side=side,mapping_mode='robot_base'),())
                    for side in ('left','right')]
        self.mode='idle';self.seq=-1;self.stamp=0.;self.source=None
        self.axial=None;self.grasp_mode='independent:0';self.grasp_generation=0
        self.active_mask=0;self.anchor_input=self.anchor_tool=self.target=None
        self.anchors=[None,None];self.robot_anchors=[None,None]
        self.current_input=self.raw_input=None

    def ingest(self, left, right, now):
        try:
            return self._ingest(left,right,now)
        except ValueError:
            for hand in self.hands:
                if hand.pose_filter:hand.pose_filter.reset()
            raise

    def _ingest(self, left, right, now):
        frames=(left,right)
        if (not math.isfinite(now) or any(
                f is None or not f.valid or f.stop or f.init or not f.source_id or f.side!=side or
                type(f.seq) is not int or f.seq<0 or not math.isfinite(f.stamp) or
                not 0<=now-f.stamp<=self.cfg['input_max_age'] or
                not all(math.isfinite(v) and 0<=v<=1 for v in (f.grip,f.trigger))
                for f,side in zip(frames,('left','right'))) or left.source_id!=right.source_id or
                left.seq!=right.seq or abs(left.stamp-right.stamp)>.02 or
                min(left.stamp,right.stamp)<=self.selected_at):
            raise ValueError('VR_PAIR_INVALID')
        for frame in frames:pose_matrix(frame.pose)
        for hand,frame in zip(self.hands,frames):
            if (hand.mode=='follow' and frame.grip>=.3 and frame.clutch_seq==hand.clutch_seq or frame.grip>.7 and
                    (hand.released or frame.clutch_seq!=hand.clutch_seq)):
                engagement_yaw(frame,self.cfg,now)
        if self.source is not None and left.source_id!=self.source:raise ValueError('VR_CONNECTION_CHANGED')
        if left.seq<=self.seq:return False
        previous=self.active_mask;mask=0;changed=False
        for i,(hand,frame) in enumerate(zip(self.hands,(left,right))):
            before=hand.generation
            hand.ingest(frame,now)
            if hand.mode=='follow':mask|=1<<i
            if hand.generation!=before:
                changed=True;self.anchors[i]=self.robot_anchors[i]=None
        self.source,self.seq,self.stamp=left.source_id,left.seq,max(left.stamp,right.stamp)
        self.current_input=np.array([hand.filtered_pose for hand in self.hands])
        self.raw_input=np.array([hand.raw_pose for hand in self.hands])
        if changed or mask!=previous:
            self.grasp_generation+=1
            self.grasp_mode='independent:'+str(mask)
            self.reanchor(preserve_active=True)
        self.active_mask=mask;self.mode='follow' if mask else 'idle'
        self.anchor_input=self.current_input.copy() if mask else None
        if self.mode=='follow' and self.anchor_tool is not None:self.target=self.mapped_target()
        return True

    def reanchor(self, *, preserve_active=False):
        # The runtime confirms hold and fences every old worker/submission
        # before capturing FK again. Clutch changes preserve the active partner;
        # an explicit pause requires both mapping baselines to be recaptured.
        self.anchor_tool=self.target=None
        if not preserve_active:
            # An explicit confirmed pause suspends both hands. Resume captures
            # fresh baselines rather than replaying motion made while paused.
            self.anchors=[None,None]
            self.robot_anchors=[None,None]
            for hand in self.hands:
                hand.mode='idle'
                hand.anchor_input=hand.anchor_robot=None
                hand.anchor_pending=False
                if hand.pose_filter:hand.pose_filter.reset()

    def capture(self, poses):
        if self.mode!='follow':raise ValueError('INDEPENDENT_ARMS_NOT_ENGAGED')
        poses=np.asarray(poses,dtype=float)
        if poses.shape!=(2,4,4):raise ValueError('INDEPENDENT_ARMS_TARGET_LAYOUT')
        for i,pose in enumerate(poses):
            pose_matrix(tuple(pose.ravel()))
            if not self.active_mask&(1<<i):
                self.anchors[i]=None;self.robot_anchors[i]=pose.copy()
            elif self.anchors[i] is None:
                # FK capture can finish after several moving input samples.
                # Re-seed only the recaptured side; the other hand's baseline
                # and filter trajectory must survive its partner's release.
                hand=self.hands[i]
                if hand.pose_filter:
                    hand.pose_filter.reset()
                    hand.filtered_pose=hand.pose_filter.update(hand.raw_pose,hand.frame.stamp)
                    self.current_input[i]=hand.filtered_pose
                self.anchors[i]=self.current_input[i].copy()
                self.robot_anchors[i]=pose.copy()
        self.anchor_input=self.current_input.copy()
        self.anchor_tool=poses.copy();self.target=self.mapped_target()

    def mapped_target(self):
        targets=self.anchor_tool.copy()
        for i in range(2):
            if self.active_mask&(1<<i):
                targets[i]=np.asarray(relative_target(self.anchors[i].ravel(),self.current_input[i].ravel(),
                    self.robot_anchors[i].ravel(),self.cfg['position_scale'],mapping_mode='robot_base',
                    operator_yaw_rad=self.hands[i].mapping_yaw)).reshape(4,4)
        return targets
