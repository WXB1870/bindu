"""Read-only, low-rate end-effector errors from device joints and mapped VR targets.

No solver residuals or commanded joints stand in for device feedback. All pose
math runs in the display process. Event identity is checked by its ROS observer.
"""
import math
from collections import deque

import numpy as np


def angle(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)/2, -1, 1))))


def pose_error(actual, target, axial=False):
    degrees = (math.degrees(math.acos(float(np.clip(actual[:3, 2] @ target[:3, 2], -1, 1))))
               if axial else angle(target[:3, :3].T @ actual[:3, :3]))
    return [float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))*1000, degrees]


def transform(value):
    value = np.asarray(value, dtype=float).reshape(4, 4)
    r = value[:3, :3]
    if (not np.isfinite(value).all() or not np.allclose(value[3], [0, 0, 0, 1]) or
            not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999):
        raise ValueError('DISPLAY_ERROR_TRANSFORM')
    return value


class EndEffectorErrors:
    """Track capture/target events; sample fresh measured FK at at most 10 Hz."""
    def __init__(self, config, fk):
        self.fk, self.config = fk, config
        self.mount = transform(config['left_to_tool'])
        right_tool = transform(config['right_to_tool'])
        axial = config['right_axial_grasp']
        axis = np.asarray(axial['axis_in_tool'], dtype=float)
        helper = np.eye(3)[np.argmin(abs(axis))]
        tangent = np.cross(helper, axis); tangent /= np.linalg.norm(tangent)
        self.tool_grip = np.eye(4)
        self.tool_grip[:3, :3] = np.column_stack((tangent, np.cross(axis, tangent), axis))
        self.tool_grip[:3, 3] = (np.linalg.inv(right_tool) @ np.r_[axial['grip_in_right_flange_m'], 1])[:3]
        self.right_grip = right_tool @ self.tool_grip
        self.left_grip = np.eye(4)
        self.left_grip[:3, 3] = config['demo']['grip_offset_m']
        self.capture = self.target = None
        self.targets = deque(maxlen=64)
        self.serial = 0
        self.last_stamp = None
        self.last = None

    def event(self, kind, payload, task_id, stamp):
        if kind == 'BIMANUAL_CAPTURED':
            # Independent-arm captures do not define a coupled reference.
            if payload.get('grasp_mode') not in ('rigid', 'right_axial'):
                return
            self.capture = self.target = None
            self.targets.clear()
            left, right = self.fk(payload['feedback'])
            self.serial += 1
            self.capture = dict(task=task_id, stamp=stamp, mode=payload['grasp_mode'],
                                relative=np.linalg.inv(left) @ right, serial=self.serial)
        elif kind == 'BIMANUAL_QP_RESULT' and self.capture and task_id == self.capture['task']:
            if payload.get('teleop_mode') != 'bimanual_tool':
                return
            target = transform(payload['target'])
            if self.target is None:self.targets.clear()
            self.target = dict(pose=target, stamp=stamp, input_stamp=payload['input_stamp'])
            self.targets.append(self.target)

    def sample(self, data, now):
        p = data.get('pico', {})
        stamp = data.get('feedback_stamp', 0)
        result = dict(valid=False, reason='等待双臂耦合接合', stamp=stamp,
                      source='关节反馈 FK', runtime_mode=data.get('runtime_mode'),
                      position_limit_mm=self.config['grasp_position_tolerance_m']*1000,
                      rotation_limit_deg=math.degrees(self.config['grasp_rotation_tolerance_rad']))
        if p.get('teleop_mode') != 'bimanual_tool':
            self.capture = self.target = self.last = None
            self.targets.clear()
            self.last_stamp = None
            return dict(result, reason='普通双臂模式')
        if (not data.get('valid') or not 0 <= now-stamp <= .2 or
                not 0 <= now-p.get('stamp', 0) <= .6):
            self.last = None
            return dict(result, reason='反馈已过期，曲线中断')
        c = self.capture
        expected = p.get('grasp_applied')
        if not c or expected != c['mode'] or p.get('phase') not in ('following', 'engaging'):
            self.last = None
            return result
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.capture = self.target = self.last = None
            return dict(result, reason='反馈时间回退，等待重新接合')
        if self.last and stamp-self.last_stamp < .1:
            return self.last
        left, right = self.fk(data['q'])
        tool = left @ self.mount
        actual_right = right @ self.right_grip
        desired_right = tool @ self.tool_grip
        coupled = pose_error(actual_right, desired_right, axial=True)
        if c['mode'] == 'rigid':
            coupled[1] = angle(c['relative'][:3, :3].T @ (np.linalg.inv(left) @ right)[:3, :3])
        result.update(valid=True, reason='', capture=f"{c['task']}:{c['stamp']}:{c['serial']}", grasp_mode=c['mode'],
                      coupled=coupled, left=None, right=None, target_stamp=None)
        # Faster QP events can always be newer than the latest vendor feedback.
        # Keep bounded history and align to feedback, rather than dropping all
        # target errors simply because the newest event is in its future.
        t = next((t for t in reversed(self.targets) if t['stamp']<=stamp),None) if self.target else None
        if (t and p.get('phase') == 'following' and
                0 <= now-t['stamp'] <= .4 and 0 <= now-t['input_stamp'] <= .25 and
                # Never apply a future target to an older feedback sample.
                0 <= stamp-t['stamp']):
            desired_left = t['pose'] @ np.linalg.inv(self.mount)
            result['left'] = pose_error(left @ self.left_grip, desired_left @ self.left_grip)
            desired_right = (t['pose'] @ self.tool_grip if c['mode'] == 'right_axial' else
                             desired_left @ c['relative'] @ self.right_grip)
            result['right'] = pose_error(actual_right, desired_right, axial=c['mode'] == 'right_axial')
            result['target_stamp'] = t['input_stamp']
        self.last_stamp, self.last = stamp, result
        return result


def pinocchio_fk(urdf, config):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    frames = [model.getFrameId(config[side+'_frame']) for side in ('left', 'right')]
    if any(frame >= model.nframes for frame in frames):
        raise ValueError('DISPLAY_ERROR_FRAME')
    def fk(joints):
        q = pin.neutral(model)
        for name in list(model.names)[1:]:
            value = joints[name]
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('DISPLAY_ERROR_JOINT')
            q[model.joints[model.getJointId(name)].idx_q] = value
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return tuple(data.oMf[frame].homogeneous.copy() for frame in frames)
    return fk
