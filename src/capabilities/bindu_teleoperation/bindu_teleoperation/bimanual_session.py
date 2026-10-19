"""Two-hand clutch and tool pose target with optional right axial support, independent of robot model."""
import math
import uuid
from dataclasses import dataclass
import numpy as np
from .vr.mapping import robot_pose, relative_target, mapping_rotation, pose_matrix
from .vr.one_euro import rotation_log, rotation_exp


class AcceptedReference:
    """Accepted position/time only; rejected or discarded proposals do not advance it.

    Times use the same control clock at capture/solve sampling, not VR event time
    or service reply time. Long gaps and clock rollback require a new capture.
    """
    def __init__(self, max_interval=.1, *, max_sample_gap=None):
        self.max_interval = float(max_interval)
        if not math.isfinite(self.max_interval) or not 0 < self.max_interval <= .1:
            raise ValueError('BIMANUAL_INTEGRATION_INTERVAL_CONFIG')
        self.max_sample_gap = self.max_interval if max_sample_gap is None else float(max_sample_gap)
        if not math.isfinite(self.max_sample_gap) or not self.max_interval <= self.max_sample_gap <= .25:
            raise ValueError('BIMANUAL_SAMPLE_GAP_CONFIG')
        self.reset()

    def reset(self):
        self.positions = None
        self.sampled_at = None
        self.input_after_capture = None

    def capture(self, positions, sampled_at):
        self.reset()
        positions = tuple(float(v) for v in positions)
        if not positions or not all(math.isfinite(v) for v in (*positions, sampled_at)):
            raise ValueError('BIMANUAL_REFERENCE_INVALID')
        self.positions, self.sampled_at = positions, sampled_at
        self.input_after_capture = sampled_at

    def permits_input(self, stamp):
        """Never submit a hand frame from before the confirmed-stop capture."""
        return (self.input_after_capture is not None and math.isfinite(stamp) and
                stamp>self.input_after_capture)

    def step(self, sampled_at):
        if self.positions is None:
            raise ValueError('BIMANUAL_REFERENCE_NOT_CAPTURED')
        dt = sampled_at-self.sampled_at
        if not math.isfinite(dt) or not 0 < dt <= self.max_sample_gap:
            raise ValueError(f'BIMANUAL_INTEGRATION_INTERVAL: elapsed={dt:.6f}s limit={self.max_sample_gap:.6f}s')
        # A bounded integration step must never consume an entire scheduling
        # stall. Sample time still records real elapsed time on acceptance.
        return self.positions, min(dt, self.max_interval)

    def accept(self, positions, sampled_at):
        self.step(sampled_at)
        positions = tuple(float(v) for v in positions)
        if len(positions) != len(self.positions) or not all(math.isfinite(v) for v in positions):
            raise ValueError('BIMANUAL_REFERENCE_INVALID')
        self.positions, self.sampled_at = positions, sampled_at


@dataclass(frozen=True)
class GraspContinuation:
    """One local handoff, issued only after cancellation and confirmed stop."""
    source: str
    clutch_seq: tuple
    grasp_mode: str
    issued_at: float
    expires_at: float


class BimanualSession:
    def __init__(self, config, generation, selected_at, *, continuation=None):
        self.control_point_in_tool=np.zeros(3)
        mapping=config.get('vr_tool_mapping')
        if mapping not in (None,'hand_line_midpoint'):
            raise ValueError('VR_TOOL_MAPPING_CONFIG')
        if mapping=='hand_line_midpoint':
            points=[(np.linalg.inv(pose_matrix(config[side+'_to_tool'])) @
                     np.r_[config['demo'][key],1.])[:3]
                    for side,key in (('left','grip_offset_m'),('right','right_grip_offset_m'))]
            axis=points[0]-points[1];spacing=float(np.linalg.norm(axis))
            if not .05<=spacing<=1.2:
                raise ValueError('VR_TOOL_GRIP_GEOMETRY')
            self.control_point_in_tool=(points[0]+points[1])/2
            config=dict(config,two_hand_rod=True,rod_axis_in_tool=(axis/spacing).tolist())
        self.cfg = config
        if type(config.get('two_hand_rod',False)) is not bool:
            raise ValueError('VR_ROD_MAPPING_CONFIG')
        if config.get('two_hand_rod'):
            axis=np.asarray(config.get('rod_axis_in_tool'),dtype=float)
            if axis.shape!=(3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis),1.):
                raise ValueError('VR_ROD_AXIS_CONFIG')
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
        self.current_hands = self.rod_anchor_hands = self.rod_anchor_basis = None
        self.rod_roll = (0.,0.)
        self.stamp = 0.
        self.input_stamps = None
        self.target = None
        self.clutch_seq = None
        self.grasp_mode = 'rigid'
        self.grasp_generation = 0
        self.axial = config.get('right_axial_grasp')
        self.trigger_armed = False
        self.continuation = continuation
        self.confirmed_hold = None
        self.pico_pair = False
        self.resumed_grasp = False
        self.engage_seconds = config['engage_seconds']
        self.resume_seconds = config.get('pico_grasp_resume_seconds',self.engage_seconds)
        if not math.isfinite(self.resume_seconds) or not 0 < self.resume_seconds <= self.engage_seconds:
            raise ValueError('BIMANUAL_GRASP_RESUME_CONFIG')
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

    @staticmethod
    def rod_swing(before, direction, transverse):
        axis=np.cross(before,direction);length=float(np.linalg.norm(axis))
        cosine=float(np.clip(before @ direction,-1.,1.))
        return (rotation_exp(axis/length*math.atan2(length,cosine)) if length>1e-8 else
                rotation_exp(transverse*math.pi) if cosine<0 else np.eye(3))

    @staticmethod
    def paired_rod_pose(hands, anchor_hands=None, anchor_basis=None, previous_roll=(0.,0.)):
        a,b=hands
        direction=a[:3,3]-b[:3,3]
        distance=float(np.linalg.norm(direction))
        if not .05 <= distance <= 1.2:
            raise ValueError('VR_GRASP_GEOMETRY')
        direction/=distance
        pose=np.eye(4);pose[:3,3]=(a[:3,3]+b[:3,3])/2
        if anchor_hands is None:
            transverse=np.eye(3)[int(np.argmin(abs(direction)))]
            transverse-=direction*float(direction @ transverse)
            transverse/=np.linalg.norm(transverse)
            pose[:3,:3]=np.column_stack((direction,transverse,np.cross(direction,transverse)))
            return pose,(0.,0.)
        swing=BimanualSession.rod_swing(anchor_basis[:,0],direction,anchor_basis[:,1])
        rolls=[]
        for hand,home,previous in zip(hands,anchor_hands,previous_roll):
            # Remove line swing, then extract only each wrist's relative twist
            # about the current rod axis. Initial wrist orientations cancel.
            vector=rotation_log(hand[:3,:3] @ home[:3,:3].T @ swing.T)
            angle=float(np.linalg.norm(vector))
            q_axis=float(direction @ vector)*(math.sin(angle/2)/angle if angle>1e-12 else .5)
            q_scalar=math.cos(angle/2)
            if math.hypot(q_axis,q_scalar)<1e-6:
                raise ValueError('VR_ROD_ROLL_DEGENERATE')
            twist=2*math.atan2(q_axis,q_scalar)
            twist=previous+math.remainder(twist-previous,2*math.pi)
            rolls.append(twist)
        pose[:3,:3]=rotation_exp(direction*sum(rolls)/2) @ swing @ anchor_basis
        return pose,tuple(rolls)

    def mapped_target(self):
        target=np.asarray(relative_target(self.anchor_input,self.current_input,self.anchor_tool,
            self.cfg['position_scale'],mapping_mode='robot_base',
            operator_yaw_rad=self.cfg['operator_yaw_rad'])).reshape(4,4)
        if self.cfg.get('two_hand_rod'):
            basis=mapping_rotation('robot_base',self.cfg['operator_yaw_rad'])
            robot_axis=self.anchor_tool[:3,:3] @ np.asarray(self.cfg['rod_axis_in_tool'])
            # Align the two rod directions ONCE at engagement. Without this
            # relative calibration a human pitch around the robot rod axis
            # could leave the robot line unchanged (or turn into axial roll).
            alignment=self.rod_swing(basis @ self.anchor_input[:3,0],robot_axis,
                                     basis @ self.anchor_input[:3,1])
            basis=alignment @ basis
            swing=self.rod_swing(self.anchor_input[:3,0],self.current_input[:3,0],self.anchor_input[:3,1])
            swing=basis @ swing @ basis.T
            rotation=swing @ self.anchor_tool[:3,:3]
            # A relative clutch permits different initial human/robot rod
            # directions. Roll must still use the ROBOT rod's current axis;
            # left-composing an input-axis roll would incorrectly tilt it.
            axis=rotation @ np.asarray(self.cfg['rod_axis_in_tool'])
            target[:3,:3]=rotation_exp(axis*sum(self.rod_roll)/2) @ rotation
        # Human translation moves the grip midpoint. Keep the real tool/CAD
        # origin unchanged and rotate it around that point, not around its tip.
        target[:3,3]+=(self.anchor_tool[:3,:3]-target[:3,:3]) @ self.control_point_in_tool
        return target

    def ingest(self, left, right, now):
        frames=(left,right)
        if (not math.isfinite(now) or
                any(f is None or not f.valid or f.stop or f.init or not f.source_id or
                type(f.seq) is not int or f.seq < 0 or not math.isfinite(f.stamp) or
                not 0 <= now-f.stamp <= self.cfg['input_max_age'] or
                f.stamp <= self.selected_at or
                not all(math.isfinite(v) and 0 <= v <= 1 for v in (f.grip,f.trigger))
                for f in frames) or left.source_id != right.source_id or
                left.side != 'left' or right.side != 'right' or
                left.seq != right.seq or abs(left.stamp-right.stamp) > .02):
            raise ValueError('VR_PAIR_INVALID')
        if self.source is not None and self.source != left.source_id:
            raise ValueError('VR_CONNECTION_CHANGED')
        if left.seq <= self.seq:
            return False
        if self.input_stamps is not None and any(
                f.stamp < previous for f, previous in zip(frames, self.input_stamps)):
            raise ValueError('VR_INPUT_UNORDERED')
        clutch_seq = (left.clutch_seq, right.clutch_seq)
        if (any(type(value) is not int or value < 0 for value in clutch_seq) or
                self.clutch_seq is not None and
                any(value < previous for value, previous in zip(clutch_seq, self.clutch_seq))):
            raise ValueError('VR_CLUTCH_UNORDERED')
        # Validate geometry before changing clutch/grasp state. A rejected
        # proposal must not consume a release counter or discard the anchor.
        if self.cfg.get('two_hand_rod'):
            hands=(robot_pose(left.pose),robot_pose(right.pose))
            engaged=self.anchor_tool is not None and min(left.grip,right.grip)>=.3 and clutch_seq==self.clutch_seq
            pose,rolls=self.paired_rod_pose(hands,self.rod_anchor_hands if engaged else None,
                self.rod_anchor_basis if engaged else None,self.rod_roll)
            self.current_hands,self.rod_roll=hands,rolls
        else:
            pose=self.paired_pose(left,right,left_orientation=self.axial is not None)
        # A nonzero counter on the first frame is historical, not a new release.
        if self.clutch_seq is not None and clutch_seq != self.clutch_seq:
            # The receiver can coalesce a brief release into the next pressed
            # frame. Enter idle even when both current grip values are high;
            # the runtime then holds and discards its pending worker result.
            self.released = True
            self.pressed_since = None
            self.mode = 'idle'
            self.anchor_input = self.anchor_tool = self.target = None
            self.confirmed_hold = self.continuation = None
            self.engage_seconds = self.cfg['engage_seconds']
            self.resumed_grasp = False
        self.clutch_seq = clutch_seq
        self.pico_pair = left.mapping == right.mapping == 'pico_snowbot_v1'
        if self.continuation is not None and (
                not self.pico_pair or self.continuation.source != left.source_id or
                self.continuation.clutch_seq != clutch_seq or
                not self.continuation.issued_at <= now <= self.continuation.expires_at):
            self.continuation = None
        desired = self.grasp_mode
        if self.axial is not None:
            if right.mapping == 'pico_snowbot_v1':
                desired = 'right_axial' if right.virtual_grip else 'rigid'
            elif right.trigger < self.release:
                self.trigger_armed = True
                desired = 'rigid'
            elif right.trigger > self.press and self.trigger_armed and min(left.grip,right.grip) > .7:
                desired = 'right_axial'
        if desired != self.grasp_mode:
            self.grasp_mode = desired
            self.grasp_generation += 1
            self.reanchor()
        self.current_input=pose
        self.source,self.seq,self.stamp=left.source_id,left.seq,max(left.stamp,right.stamp)
        self.input_stamps = (left.stamp, right.stamp)
        if min(left.grip,right.grip) < .3:
            self.released=True
            self.pressed_since=None
            self.mode='idle'
            self.anchor_input=self.anchor_tool=self.target=None
        elif min(left.grip,right.grip) > .7 and self.released:
            if self.pressed_since is None:
                self.pressed_since=self.stamp
                # Projected hands are released during the actuator handoff.
                # Physical release counters must be unchanged throughout it.
                self.resumed_grasp=bool(self.continuation is not None and
                    self.continuation.grasp_mode != self.grasp_mode and
                    self.resume_seconds < self.cfg['engage_seconds'])
                self.engage_seconds=self.resume_seconds if self.resumed_grasp else self.cfg['engage_seconds']
                self.continuation=None  # Consumed once, never a standing bypass.
            if self.mode == 'idle' and self.stamp-self.pressed_since >= self.engage_seconds:
                self.mode='follow'
                self.anchor_input=pose
                self.anchor_tool=self.target=None
        if self.mode == 'follow' and self.anchor_input is None:
            self.anchor_input = pose
        if self.mode == 'follow' and self.anchor_tool is not None:
            self.target=self.mapped_target()
        return True

    def reanchor(self):
        """Discard pending mapping at a confirmed-stop constraint transition."""
        self.anchor_input = self.anchor_tool = self.target = None
        self.rod_anchor_hands = self.rod_anchor_basis = None
        self.rod_roll = (0.,0.)

    def capture(self, tool_pose):
        if self.mode != 'follow' or self.anchor_input is None:
            raise ValueError('BIMANUAL_NOT_ENGAGED')
        # Capture is asynchronous. Re-latch the controller on completion too,
        # so motion while the worker captures cannot turn into a first target
        # jump. The robot is held until this capture has completed.
        self.anchor_input=self.current_input.copy()
        if self.cfg.get('two_hand_rod'):
            self.rod_anchor_hands=tuple(hand.copy() for hand in self.current_hands)
            self.rod_anchor_basis=self.anchor_input[:3,:3].copy()
            self.rod_roll=(0.,0.)
        self.anchor_tool=np.asarray(tool_pose,dtype=float).reshape(4,4)
        self.target=self.mapped_target()
        self.confirmed_hold=(self.source,self.clutch_seq,self.grasp_mode) if self.pico_pair else None

    def stopped_continuation(self, now):
        """Runtime calls only after a canceled goal's fresh, confirmed stop.

        Keep the physical hold identity, never a tool target or grasp anchor.
        Slow handoffs fall back to the normal hold; source/release changes do too.
        """
        if (self.confirmed_hold != (self.source,self.clutch_seq,self.grasp_mode) or
                not 0 <= now-self.stamp <= self.cfg['input_max_age']):
            return None
        return GraspContinuation(self.source,self.clutch_seq,self.grasp_mode,now,now+1.)


class ToolPoseSession:
    """Mapped teleoperation input: tool pose + right-grip state, no phase clock."""
    def __init__(self, config, generation, selected_at):
        self.cfg, self.generation, self.selected_at = config, generation, selected_at
        self.identity = uuid.uuid4().hex
        self.source = None
        self.seq, self.stamp = -1, 0.
        self.mode = 'idle'
        self.grasp_mode, self.grasp_generation = 'rigid', 0
        self.axial = config.get('right_axial_grasp')
        self.anchor_input = self.anchor_tool = self.target = self.current_input = None

    def ingest(self, frame, now):
        if (frame is None or not frame.valid or frame.stop or not frame.source_id or
                not math.isfinite(now) or not math.isfinite(frame.stamp) or
                not 0 <= now-frame.stamp <= self.cfg['input_max_age'] or
                frame.stamp <= self.selected_at or type(frame.tight) is not bool or
                type(frame.seq) is not int or frame.seq < 0 or
                frame.frame_id != self.cfg['input_frame']):
            raise ValueError('TOOL_INPUT_INVALID')
        if self.source is not None and self.source != frame.source_id:
            raise ValueError('TOOL_CONNECTION_CHANGED')
        if frame.seq <= self.seq:
            return False
        if self.source is not None and frame.stamp < self.stamp:
            raise ValueError('TOOL_INPUT_UNORDERED')
        pose = np.asarray(frame.pose, dtype=float).reshape(4,4)
        rotation = pose[:3,:3]
        if (not np.isfinite(pose).all() or not np.allclose(pose[3], [0,0,0,1], atol=1e-8) or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or
                np.linalg.det(rotation) < .999999):
            raise ValueError('TOOL_POSE_INVALID')
        desired = 'rigid' if frame.tight else 'right_axial'
        if desired == 'right_axial' and self.axial is None:
            raise ValueError('TOOL_AXIAL_UNCONFIGURED')
        if desired != self.grasp_mode:
            self.grasp_mode = desired
            self.grasp_generation += 1
            self.reanchor()
        self.source,self.seq,self.stamp = frame.source_id,frame.seq,frame.stamp
        self.current_input = self.anchor_input = pose.copy()
        self.target = pose.copy()
        self.mode = 'follow'
        return True

    def reanchor(self):
        # Rebuild the measured grasp after stopping, but never shift an
        # absolute tool target when the operator changes grip state.
        self.anchor_tool = None

    def capture(self, tool_pose):
        if self.mode != 'follow' or self.current_input is None:
            raise ValueError('TOOL_NOT_ENGAGED')
        self.anchor_tool = np.asarray(tool_pose, dtype=float).reshape(4,4)
        self.target = self.current_input.copy()


class ToolPause:
    """Explicit pause is latched; resume needs confirmed stop and a new input."""
    def __init__(self):
        self.phase = 'follow'
        self.resume_after = float('-inf')

    def request(self, paused, now):
        if type(paused) is not bool or not math.isfinite(now):
            raise ValueError('TOOL_PAUSE_INVALID')
        if paused:
            if self.phase != 'paused':
                self.phase = 'pausing'
        elif self.phase == 'paused':
            self.phase = 'waiting_input'
            self.resume_after = now
        else:
            raise ValueError('TOOL_PAUSE_NOT_CONFIRMED')

    def confirm_stopped(self):
        if self.phase != 'pausing':
            raise ValueError('TOOL_PAUSE_NOT_REQUESTED')
        self.phase = 'paused'

    def accepts(self, stamp):
        if self.phase == 'waiting_input' and math.isfinite(stamp) and stamp > self.resume_after:
            self.phase = 'follow'
        return self.phase == 'follow'
