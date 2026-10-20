"""Observed scan raster for LIO map coordinates; unseen cells remain unknown."""
import math


class ObservedGrid:
    def __init__(self, resolution, hit_padding, max_side):
        self.resolution, self.padding, self.max_side = resolution, hit_padding, max_side
        self.cells = {}; self.bounds = None

    def add(self, origin, ends):
        r = self.resolution
        start = tuple(math.floor(x/r) for x in origin)
        radius = math.ceil(self.padding/r)
        ends = [tuple(math.floor(x/r) for x in p) for p in ends]
        if not ends: return
        points = [start, *ends]
        bounds = [min(p[0] for p in points)-radius, min(p[1] for p in points)-radius,
                  max(p[0] for p in points)+radius, max(p[1] for p in points)+radius]
        if self.bounds:
            bounds = [min(bounds[0],self.bounds[0]),min(bounds[1],self.bounds[1]),
                      max(bounds[2],self.bounds[2]),max(bounds[3],self.bounds[3])]
        if max(bounds[2]-bounds[0]+1, bounds[3]-bounds[1]+1) > self.max_side:
            raise ValueError('GRID_BOUNDS_EXCEEDED: 新批次或明确调整grid.max_side；不截断地图')
        self.bounds = bounds
        for end in ends:
            dx, dy = end[0]-start[0], end[1]-start[1]
            steps = max(abs(dx), abs(dy), 1)
            for i in range(steps):
                cell = (round(start[0]+dx*i/steps),round(start[1]+dy*i/steps))
                self.cells.setdefault(cell, 0)
            for x in range(end[0]-radius,end[0]+radius+1):
                for y in range(end[1]-radius,end[1]+radius+1): self.cells[x,y] = 100

    def raster(self):
        if not self.bounds: raise ValueError('NO_OBSERVED_MAPPING_DATA')
        x0,y0,x1,y1 = self.bounds
        w,h = x1-x0+1,y1-y0+1
        data = [-1]*(w*h)
        for (x,y), value in self.cells.items(): data[(y-y0)*w+x-x0] = value
        return w,h,[x0*self.resolution,y0*self.resolution],data


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); node = rclpy.create_node('lio_observed_grid')
    get = lambda key, value: node.declare_parameter(key, value).value
    grid = ObservedGrid(get('resolution', .05), get('hit_padding', .15), get('max_side', 2048))
    frame = get('map_frame','map'); max_age = get('max_age',.5)
    publisher = node.create_publisher(OccupancyGrid,'navigation/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer(); listener = TransformListener(buffer,node)
    pending = None; last_stamp = 0.; fault = ''
    def receive(message):
        nonlocal pending
        pending = message
    node.create_subscription(LaserScan,'navigation/scan',receive,qos_profile_sensor_data)
    def update():
        nonlocal pending, last_stamp, fault
        if fault or pending is None: return
        scan = pending; t = scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
        now = node.get_clock().now().nanoseconds/1e9
        if not 0 <= now-t <= max_age or t <= last_stamp:
            pending = None; return
        try:
            tr = buffer.lookup_transform(frame,scan.header.frame_id,rclpy.time.Time.from_msg(scan.header.stamp)).transform
        except Exception: return  # Wait briefly for the acquisition-time TF, never use latest instead.
        pending = None
        p,q = tr.translation,tr.rotation
        # Full SE(3) transform for a pitched sensor, not a guessed planar origin.
        import numpy as np
        from bindu_lio.localization import pose_matrix
        matrix = pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        ends = []
        for i,d in enumerate(scan.ranges):
            if not math.isfinite(d) or not scan.range_min <= d <= scan.range_max: continue
            a = scan.angle_min+i*scan.angle_increment
            point = matrix @ np.array([d*math.cos(a),d*math.sin(a),0.,1.])
            ends.append(point[:2])
        try:
            grid.add([p.x,p.y],ends)
            w,h,origin,data = grid.raster()
        except ValueError as exc:
            fault = str(exc); node.get_logger().error(fault); return
        last_stamp = t
        msg = OccupancyGrid(); msg.header = scan.header; msg.header.frame_id = frame
        msg.info.resolution = grid.resolution; msg.info.width = w; msg.info.height = h
        msg.info.origin.position.x,msg.info.origin.position.y = origin
        msg.info.origin.orientation.w = 1.; msg.data = data
        publisher.publish(msg)
    node.create_timer(1.,update)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
