"""Sensor evidence and observed-ray occupancy grid for the shared physics test."""
import base64
import json
import math
from pathlib import Path
import numpy as np
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String
from sensor_msgs_py.point_cloud2 import read_points_numpy


class LioEvidence:
    def __init__(self, node, namespace, qos, raw, seen):
        self.frames = []
        self.collect = True
        self.odometry = []
        self.seen = seen
        self.raw = raw
        for topic, kind in [('lio/points', PointCloud2), ('lio/cloud_registered', PointCloud2),
                            ('lio/imu', Imu), ('lio/status', String), ('lio/localization_status', String)]:
            node.create_subscription(kind, namespace+'/'+topic, lambda m, t=topic:self.record(t, m), qos)

    def record(self, topic, message):
        from rosidl_runtime_py.convert import message_to_ordereddict
        import time
        self.seen[topic] = message
        if isinstance(message, PointCloud2):
            data = {'header': message_to_ordereddict(message.header), 'height': message.height,
                    'width': message.width, 'point_step': message.point_step, 'row_step': message.row_step,
                    'fields': [message_to_ordereddict(f) for f in message.fields],
                    'is_bigendian': message.is_bigendian, 'data_base64': base64.b64encode(bytes(message.data)).decode()}
            if self.collect and topic == 'lio/cloud_registered':
                stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
                points = read_points_numpy(message, field_names=('x','y','z'), skip_nans=True).reshape(-1,3).copy()
                self.frames.append((stamp, points))
        else:
            data = message_to_ordereddict(message)
        self.raw.write(json.dumps({'topic': topic, 'received': time.time(), 'data': data})+'\n')

    def save_grid(self, output, odometry, resolution=.05):
        """Observed free rays and occupied height slice, never scene geometry.

        Current fixture mounts the laser directly above base, so planar ray
        origin equals base x/y. Each endpoint represents a 0.2m LIO voxel;
        mark its 0.1m half-width conservatively instead of leaving false gaps.
        """
        from PIL import Image, ImageDraw, ImageFilter
        if not self.frames or not odometry:
            raise ValueError('NO_OBSERVED_MAPPING_DATA')
        stamps = np.array([p['stamp'] for p in odometry])
        observations = []
        for stamp, points in self.frames:
            i = int(np.argmin(abs(stamps-stamp)))
            if abs(stamps[i]-stamp) > .02:
                continue
            points = points[(points[:,2]>=.1)&(points[:,2]<=.95)]
            if len(points):observations.append((odometry[i], points[:,:2]))
        points = np.concatenate([p for _,p in observations])
        origin = np.floor((points.min(axis=0)-.25)/resolution)*resolution
        size = np.ceil((points.max(axis=0)+.25-origin)/resolution).astype(int)
        if np.any(size>2000):raise ValueError('GRID_BOUNDS_EXCEEDED')
        image = Image.new('L', tuple(size), 205)
        draw = ImageDraw.Draw(image)
        hits = np.zeros(tuple(size[::-1]), dtype=np.uint8)
        for pose, points in observations:
            start = tuple(np.floor((np.array([pose['x'],pose['y']])-origin)/resolution).astype(int))
            ends = np.floor((points-origin)/resolution).astype(int)
            for end in ends:draw.line([start, tuple(end)],fill=254)
            hits[ends[:,1],ends[:,0]] = 255
        occupied = np.asarray(Image.fromarray(hits).filter(ImageFilter.MaxFilter(5)))>0
        cells = np.asarray(image).copy();cells[occupied]=0
        image = Image.fromarray(cells[::-1]);image.save(output/'room.pgm')
        import yaml
        (output/'room.yaml').write_text(yaml.safe_dump({'image':'room.pgm','mode':'trinary','resolution':resolution,
            'origin':[float(origin[0]),float(origin[1]),0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
        grid={'info':{'width':int(size[0]),'height':int(size[1]),'resolution':resolution,
                     'origin':{'position':{'x':float(origin[0]),'y':float(origin[1]),'z':0.}}},
              'data':np.where(cells==0,100,np.where(cells==254,0,-1)).ravel().tolist()}
        (output/'slam_grid.json').write_text(json.dumps(grid))
        return {'width':int(size[0]),'height':int(size[1]),'resolution':resolution,
                'occupied_cells':int(np.sum(cells==0)),'free_cells':int(np.sum(cells==254)),
                'method':'measured LIO registered rays, z slice 0.1..0.95m, 0.1m voxel half-width; no simulator map'}
