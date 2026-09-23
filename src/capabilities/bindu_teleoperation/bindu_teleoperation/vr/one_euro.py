"""Speed-adaptive pose low-pass, following https://gery.casiez.net/1euro/.

Position uses metres; orientation uses a shortest-arc SO(3) extension in radians,
not Euler angles or elementwise matrix filtering. No ROS, IK or joint smoothing.
"""
import math
import numpy as np
from .mapping import pose_matrix


def validate_config(config):
    if not isinstance(config, dict) or type(config.get('enabled')) is not bool:
        raise ValueError('VR_INVALID_FILTER_CONFIG')
    for part in ('position', 'rotation'):
        if not isinstance(config.get(part), dict):
            raise ValueError('VR_INVALID_FILTER_CONFIG')
        for key in ('min_cutoff_hz', 'beta', 'derivative_cutoff_hz'):
            value = config.get(part, {}).get(key)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or (value < 0 if key == 'beta' else value <= 0)):
                raise ValueError('VR_INVALID_FILTER_CONFIG')


def alpha(cutoff, dt):
    return 1. / (1. + 1. / (2. * math.pi * cutoff * dt))


def rotation_log(r):
    # Symmetric quaternion extraction remains stable at a half turn.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])
    q = np.linalg.eigh(k)[1][:, -1]
    if q[3] < 0:
        q = -q
    length = np.linalg.norm(q[:3])
    return 2*q[:3] if length < 1e-12 else q[:3]*(2*math.atan2(length, q[3])/length)


def rotation_exp(v):
    angle = np.linalg.norm(v)
    x, y, z = v
    skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    a = math.sin(angle)/angle if angle > 1e-8 else 1.-angle**2/6
    b = (1.-math.cos(angle))/angle**2 if angle > 1e-8 else .5-angle**2/24
    return np.eye(3)+a*skew+b*(skew@skew)


class OneEuroPose:
    def __init__(self, config):
        validate_config(config)
        self.config = config
        self.reset()

    def reset(self):
        self.stamp = self.raw = self.filtered = None
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def update(self, pose, stamp):
        pose = pose_matrix(pose)
        if not math.isfinite(stamp) or (self.stamp is not None and stamp <= self.stamp):
            raise ValueError('VR_FILTER_TIME_ORDER')
        if self.stamp is None or not self.config['enabled']:
            self.filtered = pose.copy()
        else:
            dt = stamp-self.stamp
            pc, rc = self.config['position'], self.config['rotation']
            da = alpha(pc['derivative_cutoff_hz'], dt)
            self.velocity += da*((pose[:3,3]-self.raw[:3,3])/dt-self.velocity)
            da = alpha(rc['derivative_cutoff_hz'], dt)
            self.angular_velocity += da*(rotation_log(pose[:3,:3]@self.raw[:3,:3].T)/dt-self.angular_velocity)
            pa = alpha(pc['min_cutoff_hz']+pc['beta']*np.linalg.norm(self.velocity), dt)
            ra = alpha(rc['min_cutoff_hz']+rc['beta']*np.linalg.norm(self.angular_velocity), dt)
            self.filtered[:3,3] += pa*(pose[:3,3]-self.filtered[:3,3])
            self.filtered[:3,:3] = rotation_exp(ra*rotation_log(pose[:3,:3]@self.filtered[:3,:3].T))@self.filtered[:3,:3]
        self.raw, self.stamp = pose, stamp
        return self.filtered.copy()
