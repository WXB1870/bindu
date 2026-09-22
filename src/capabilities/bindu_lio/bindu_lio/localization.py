"""ROS-independent adaptation of the supplied FAST_LIO_LOCALIZATION ICP.

Source: historical global_localization.py (coarse x5, fine x1, point-to-point).
Unlike the source, distances use a radial crop, inputs are immutable snapshots,
and both fitness and residual error must pass. This is seeded registration,
not arbitrary-position place recognition.
"""
from dataclasses import dataclass
import math
import numpy as np


def rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)):
        raise ValueError('INVALID_RIGID_TRANSFORM')
    return matrix.copy()


def inverse_se3(value):
    matrix = rigid_transform(value)
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def pose_matrix(position, quaternion):
    xyz, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if xyz.shape != (3,) or q.shape != (4,) or not np.isfinite(xyz).all() or not np.isfinite(q).all():
        raise ValueError('INVALID_POSE')
    if not np.isclose(np.linalg.norm(q), 1., atol=1e-5):
        raise ValueError('INVALID_QUATERNION')
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = xyz
    return rigid_transform(result)


def initial_map_from_odom(map_from_base, odom_from_base):
    return rigid_transform(map_from_base) @ inverse_se3(odom_from_base)


def correction_distance(previous, current):
    delta = inverse_se3(previous) @ rigid_transform(current)
    return (float(np.linalg.norm(delta[:3, 3])),
            math.acos(float(np.clip((np.trace(delta[:3, :3])-1.)/2., -1., 1.))))


@dataclass(frozen=True)
class RegistrationConfig:
    map_voxel: float = .2
    scan_voxel: float = .1
    correspondence: float = .3
    min_fitness: float = .8
    max_rmse: float = .12
    crop_radius: float = 30.
    max_translation_step: float = .5
    max_rotation_step: float = .35
    min_points: int = 30

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('INVALID_REGISTRATION_' + name.upper())
        if self.min_fitness > 1. or not isinstance(self.min_points, int):
            raise ValueError('INVALID_REGISTRATION_THRESHOLDS')


class ScanMapRegistration:
    def __init__(self, points, config=RegistrationConfig()):
        import open3d as o3d
        self.o3d, self.config = o3d, config
        cloud = self._cloud(points)
        self.map = cloud.voxel_down_sample(config.map_voxel)
        if len(self.map.points) < config.min_points:
            raise ValueError('MAP_TOO_SMALL')

    def _cloud(self, points):
        array = np.asarray(points, dtype=float)
        if array.ndim != 2 or array.shape[1] != 3 or not np.isfinite(array).all():
            raise ValueError('INVALID_POINT_CLOUD')
        cloud = self.o3d.geometry.PointCloud()
        cloud.points = self.o3d.utility.Vector3dVector(array)
        return cloud

    def match(self, scan_in_odom, odom_from_base, initial):
        cfg = self.config
        initial = rigid_transform(initial)
        scan = self._cloud(scan_in_odom)
        center = (initial @ rigid_transform(odom_from_base))[:3, 3]
        points = np.asarray(self.map.points)
        submap = self._cloud(points[np.linalg.norm(points-center, axis=1) < cfg.crop_radius])
        if min(len(scan.points), len(submap.points)) < cfg.min_points:
            raise ValueError('INSUFFICIENT_REGISTRATION_POINTS')
        registration = self.o3d.pipelines.registration
        transform = initial
        for scale in (5., 1.):
            source = scan.voxel_down_sample(cfg.scan_voxel * scale)
            target = submap.voxel_down_sample(cfg.map_voxel * scale)
            if min(len(source.points), len(target.points)) < cfg.min_points:
                raise ValueError('INSUFFICIENT_VOXEL_POINTS')
            result = registration.registration_icp(
                source, target, cfg.correspondence * scale, transform,
                registration.TransformationEstimationPointToPoint(),
                registration.ICPConvergenceCriteria(max_iteration=20))
            transform = rigid_transform(result.transformation)
        translation, rotation = correction_distance(initial, transform)
        if (result.fitness < cfg.min_fitness or not math.isfinite(result.inlier_rmse)
                or result.inlier_rmse > cfg.max_rmse
                or translation > cfg.max_translation_step or rotation > cfg.max_rotation_step):
            raise ValueError('REGISTRATION_REJECTED')
        return transform, float(result.fitness), float(result.inlier_rmse)
