"""Wheel encoder odometry, independent of robot names and ROS.

Encoders may wrap by 2*pi; each wheel must travel less than pi per sample.
"""
import math


class DifferentialOdometry:
    def __init__(self, radius, separation):
        if not all(math.isfinite(v) and v > 0 for v in (radius, separation)):
            raise ValueError('INVALID_WHEEL_GEOMETRY')
        self.radius, self.separation = radius, separation
        self.previous = None
        self.x = self.y = self.yaw = 0.

    def update(self, left, right, time):
        if not all(math.isfinite(v) for v in (left, right, time)):
            raise ValueError('INVALID_WHEEL_SAMPLE')
        if self.previous is None:
            self.previous = (left, right, time)
            return self.x, self.y, self.yaw, 0., 0.
        old_left, old_right, old_time = self.previous
        if time <= old_time:
            raise ValueError('ODOMETRY_TIME_NOT_INCREASING')
        dl = math.remainder(left-old_left,2*math.pi)*self.radius
        dr = math.remainder(right-old_right,2*math.pi)*self.radius
        distance, turn = (dl+dr)/2, (dr-dl)/self.separation
        half = turn/2
        chord = distance*(math.sin(half)/half if half else 1.)
        self.x += chord*math.cos(self.yaw+half)
        self.y += chord*math.sin(self.yaw+half)
        self.yaw = math.atan2(math.sin(self.yaw+turn), math.cos(self.yaw+turn))
        self.previous = (left, right, time)
        return self.x, self.y, self.yaw, distance/(time-old_time), turn/(time-old_time)
