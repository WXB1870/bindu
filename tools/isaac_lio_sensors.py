"""PhysX ray lidar and finite-difference IMU, independent of robot joint names.

Truth is used only to synthesize sensor measurements. No odometry, map or TF is
published here. Acquisition uses the existing bridge's wall-clock time; IMU
kinematics use that same time, including changes in simulation real-time factor.
"""
import math
import numpy as np


def rotation_xyzw(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class InertialSampler:
    """Backward differences at the IMU origin; SI specific force, body gyro."""
    def __init__(self):
        self.previous = None
        self.velocity = None

    def sample(self, stamp, position, rotation):
        acceleration, angular = np.zeros(3), np.zeros(3)
        if self.previous is not None:
            old_time, old_position, old_rotation = self.previous
            dt = stamp-old_time
            if dt <= 0:
                raise ValueError('IMU_TIME_NOT_INCREASING')
            velocity = (position-old_position)/dt
            if self.velocity is not None:
                acceleration = (velocity-self.velocity)/dt
            self.velocity = velocity
            delta = old_rotation.T @ rotation
            angular = np.array([delta[2, 1]-delta[1, 2], delta[0, 2]-delta[2, 0],
                                delta[1, 0]-delta[0, 1]])/(2*dt)
        self.previous = (stamp, position.copy(), rotation.copy())
        return rotation.T @ (acceleration-np.array([0., 0., -9.81])), angular


class LioSensors:
    def __init__(self, node, config, excluded_path):
        from sensor_msgs.msg import PointCloud2, PointField, Imu
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.config, self.excluded = node, config, excluded_path
        self.cfg = config['simulation']
        self.frames = config['frames']
        self.Cloud, self.Imu = PointCloud2, Imu
        self.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                       for i, n in enumerate(('x', 'y', 'z', 'intensity', 'time'))]
        self.cloud = node.create_publisher(PointCloud2, config['points_topic'], qos_profile_sensor_data)
        self.imu = node.create_publisher(Imu, config['imu_topic'], qos_profile_sensor_data)
        self.fault = ''
        node.create_subscription(String, 'navigation/sensor_fault', self.set_fault, 1)
        self.random = np.random.default_rng(self.cfg['random_seed'])
        self.inertial = InertialSampler()
        self.step, self.points, self.start = 0, [], None
        self.imu_from_base = self.transform(config['imu_from_base'])
        self.imu_from_lidar = self.transform(config['imu_from_lidar'])
        self.base_from_imu = np.linalg.inv(self.imu_from_base)
        self.base_from_lidar = self.base_from_imu @ self.imu_from_lidar
        bins, steps = self.cfg['azimuth_bins'], self.cfg['steps_per_scan']
        if not 8 <= steps <= bins or bins % steps:
            raise ValueError('AZIMUTH_BINS_MUST_DIVIDE_PHYSICS_STEPS')
        for key in ('range_noise_std', 'accel_noise_std', 'gyro_noise_std'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] < 0:
                raise ValueError('INVALID_SENSOR_NOISE')
        if not 0 < self.cfg['min_range'] < self.cfg['max_range']:
            raise ValueError('INVALID_LIDAR_RANGE')
        directions = []
        for azimuth in np.arange(bins)*2*math.pi/bins-math.pi:
            for elevation in np.deg2rad(self.cfg['elevations_degrees']):
                directions.append([math.cos(elevation)*math.cos(azimuth),
                                   math.cos(elevation)*math.sin(azimuth), math.sin(elevation)])
        self.directions = np.array(directions).reshape(steps, -1, 3)

    @staticmethod
    def transform(values):
        values=np.asarray(values,dtype=float)
        if values.shape!=(7,) or not np.isfinite(values).all() or not np.isclose(np.linalg.norm(values[3:]),1.,atol=1e-6):
            raise ValueError('INVALID_SENSOR_TRANSFORM')
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_xyzw(values[3:])
        matrix[:3, 3] = values[:3]
        return matrix

    def set_fault(self, message):
        self.fault = message.data
        self.points, self.start, self.step = [], None, 0

    def publish(self, feedback):
        from omni.physx import get_physx_scene_query_interface
        stamp = feedback.stamp.sec+feedback.stamp.nanosec/1e9
        p, q = feedback.base_pose.position, feedback.base_pose.orientation
        base = np.eye(4)
        base[:3, :3] = rotation_xyzw([q.x, q.y, q.z, q.w])
        base[:3, 3] = [p.x, p.y, p.z]
        imu_pose = base @ self.base_from_imu
        acceleration, angular = self.inertial.sample(stamp, imu_pose[:3, 3], imu_pose[:3, :3])
        if self.fault != 'imu_loss':
            imu = self.Imu()
            imu.header.stamp, imu.header.frame_id = feedback.stamp, self.frames['imu']
            imu.orientation_covariance[0] = -1.  # Orientation is not measured/provided.
            acceleration += self.random.normal(0., self.cfg['accel_noise_std'], 3)
            angular += self.random.normal(0., self.cfg['gyro_noise_std'], 3)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, acceleration)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, angular)
            for i in (0, 4, 8):
                imu.linear_acceleration_covariance[i] = self.cfg['accel_noise_std']**2
                imu.angular_velocity_covariance[i] = self.cfg['gyro_noise_std']**2
            self.imu.publish(imu)
        if self.fault in ('scan_loss', 'pose_loss'):
            return
        if self.start is None:
            self.start, self.start_stamp = stamp, feedback.stamp
        lidar = base @ self.base_from_lidar
        query = get_physx_scene_query_interface()
        origin = tuple(map(float, lidar[:3, 3]))
        for direction in self.directions[self.step]:
            hits = []
            def hit(value):
                if not str(value.collision).startswith(self.excluded+'/'):
                    hits.append(float(value.distance))
                return True
            query.raycast_all(origin, tuple(map(float, lidar[:3, :3] @ direction)), self.cfg['max_range'], hit)
            if not hits:
                continue
            distance = min(hits)+self.random.normal(0., self.cfg['range_noise_std'])
            if self.cfg['min_range'] <= distance <= self.cfg['max_range']:
                self.points.append([*(direction*distance), 1., stamp-self.start])
        self.step += 1
        if self.step == self.cfg['steps_per_scan']:
            if self.points:
                message = self.Cloud()
                message.header.stamp, message.header.frame_id = self.start_stamp, self.frames['lidar']
                message.height, message.width = 1, len(self.points)
                message.fields, message.point_step = self.fields, 20
                message.row_step, message.is_dense = 20*len(self.points), True
                message.data = np.asarray(self.points, dtype='<f4').tobytes()
                self.cloud.publish(message)
            self.step, self.points, self.start = 0, [], None
