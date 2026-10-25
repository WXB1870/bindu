"""Adapted from v3.4 wrapper.py and huawei_robot_server_new.py.

Preserves the OpenXR basis and optional v3.4 composition. The robot-base mode
keeps direction calibration independent of clutch origins. No filtering or IK.
"""
import math
import numpy as np

OPENXR_TO_ROBOT = np.array([[0., 0., -1., 0.], [-1., 0., 0., 0.],
                            [0., 1., 0., 0.], [0., 0., 0., 1.]])


def pose_matrix(values):
    a = np.asarray(values, dtype=float)
    if a.size != 16 or not np.isfinite(a).all():
        raise ValueError('VR_INVALID_POSE')
    a = a.reshape(4, 4)
    r = a[:3, :3]
    if (not np.allclose(a[3], [0, 0, 0, 1], atol=1e-5) or
            not np.allclose(r.T @ r, np.eye(3), atol=1e-3) or
            not np.isclose(np.linalg.det(r), 1., atol=1e-3)):
        raise ValueError('VR_INVALID_ROTATION')
    # Browser/XR matrices can be near rotations at float precision. Keep the
    # rejection boundary above, then return a rigid transform so relative
    # composition cannot amplify small scale/shear into an invalid QP target.
    result = a.copy()
    u, _, vt = np.linalg.svd(r)
    result[:3,:3] = u @ vt
    result[3] = [0.,0.,0.,1.]
    return result


def robot_pose(openxr_pose):
    return OPENXR_TO_ROBOT @ pose_matrix(openxr_pose) @ OPENXR_TO_ROBOT.T


def mapping_rotation(mode, yaw):
    """Fixed rotation after OpenXR conversion; positive yaw maps +X toward +Y."""
    if mode not in ('v34_anchor', 'robot_base'):
        raise ValueError('VR_INVALID_MAPPING_MODE')
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        raise ValueError('VR_INVALID_OPERATOR_YAW')
    if mode == 'v34_anchor' and yaw != 0.:
        raise ValueError('VR_YAW_REQUIRES_ROBOT_BASE')
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def direction_reference(config):
    mode=config.get('direction_reference','fixed')
    if mode not in ('fixed','headset_on_engage'):
        raise ValueError('VR_DIRECTION_REFERENCE_CONFIG')
    if mode=='headset_on_engage' and config.get('mapping_mode','robot_base')!='robot_base':
        raise ValueError('VR_HEADSET_REQUIRES_ROBOT_BASE')
    mapping_rotation('robot_base',config.get('operator_yaw_rad',0.))
    return mode


def headset_pose(frame, config, now):
    """Validate a tracked sample without requiring a horizontal gaze."""
    if (not frame.head_pose or frame.head_source_id!=frame.source_id or
            type(frame.head_seq) is not int or frame.head_seq<1 or
            not all(type(v) in (int,float) and math.isfinite(v) for v in (now,frame.head_stamp,frame.head_age_s)) or
            frame.head_stamp>frame.stamp or not 0<=frame.head_age_s<=config['input_max_age'] or
            not 0<=now-frame.head_stamp<=config['input_max_age'] or
            now-frame.head_stamp+frame.head_age_s>config['input_max_age']):
        raise ValueError('VR_HEADSET_UNAVAILABLE')
    return robot_pose(frame.head_pose)


def engagement_yaw(frame, config, now):
    """Human forward becomes base +X; remove pitch/roll, preserve fixed trim."""
    trim=config.get('operator_yaw_rad',0.)
    if direction_reference(config)=='fixed':return trim
    head=headset_pose(frame,config,now)
    forward=head[:2,0]  # Local OpenXR -Z is local robot +X.
    if np.linalg.norm(forward)<.1:
        raise ValueError('VR_HEADSET_HEADING_UNDEFINED')
    return trim-math.atan2(float(forward[1]),float(forward[0]))


def relative_target(anchor_input, current_input, anchor_robot, scale=1., *,
                    mapping_mode='v34_anchor', operator_yaw_rad=0.):
    if isinstance(scale,bool) or not isinstance(scale,(int,float)) or not math.isfinite(scale) or scale<=0:
        raise ValueError('VR_INVALID_POSITION_SCALE')
    basis = mapping_rotation(mapping_mode, operator_yaw_rad)
    anchor, current = pose_matrix(anchor_input), pose_matrix(current_input)
    robot = pose_matrix(anchor_robot)
    target = np.eye(4)
    if mapping_mode == 'v34_anchor':
        delta = np.linalg.inv(anchor) @ current
        translation, rotation = delta[:3, 3], delta[:3, :3]
    else:
        # Spatial increments expressed in a fixed frame, not the initial wrist
        # axes. Conjugation applies the same calibration to rotation and motion.
        translation = basis @ (current[:3, 3] - anchor[:3, 3])
        rotation = basis @ current[:3, :3] @ anchor[:3, :3].T @ basis.T
    target[:3, 3] = robot[:3, 3] + scale * translation
    target[:3, :3] = rotation @ robot[:3, :3]
    return tuple(target.ravel())
