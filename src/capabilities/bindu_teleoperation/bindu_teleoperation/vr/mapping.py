"""Adapted from v3.4 wrapper.py and huawei_robot_server_new.py.

Preserves its OpenXR basis and relative position/orientation composition. Robot
offsets and scale are configuration, not SDK or hardware assumptions.
"""
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
    return a.copy()


def robot_pose(openxr_pose):
    return OPENXR_TO_ROBOT @ pose_matrix(openxr_pose) @ OPENXR_TO_ROBOT.T


def relative_target(anchor_input, current_input, anchor_robot, scale=1.):
    delta = np.linalg.inv(pose_matrix(anchor_input)) @ pose_matrix(current_input)
    robot = pose_matrix(anchor_robot)
    target = np.eye(4)
    target[:3, 3] = robot[:3, 3] + scale * delta[:3, 3]
    target[:3, :3] = delta[:3, :3] @ robot[:3, :3]
    return tuple(target.ravel())
