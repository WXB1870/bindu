"""Optional, robot-independent geometric room and PhysX planar range sensor.

Consumes measured pose/twist from a simulator adapter. No joint names, URDF,
wheel kinematics or navigation algorithms belong here. Ideal odometry is explicit.
"""
import random
import math


def add_room(world, config):
    from pxr import UsdGeom, UsdPhysics, Gf
    for row in config['boxes']:
        box = UsdGeom.Cube.Define(world.stage, '/World/Room/'+row['name'])
        box.CreateSizeAttr(1.)
        box.AddTranslateOp().Set(Gf.Vec3d(*row['center']))
        box.AddScaleOp().Set(Gf.Vec3d(*row['size']))
        box.CreateDisplayColorAttr([Gf.Vec3f(*row.get('color', [.55,.6,.65]))])
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())


class NavigationSensors:
    def __init__(self, node, world, config, robot, excluded_path, sensor_config=None):
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import String
        self.node, self.world, self.cfg, self.robot = node, world, config, robot
        self.excluded_path = excluded_path
        self.sensor_config=sensor_config or {'mode':'ideal'}
        self.external=self.sensor_config['mode']=='wheel'
        if self.sensor_config['mode'] not in ('ideal','wheel'):raise ValueError('INVALID_ODOMETRY_MODE')
        self.random=random.Random(self.sensor_config.get('random_seed',0))
        self.noise=self.sensor_config.get('range_noise_std',0.)
        if not math.isfinite(self.noise) or self.noise<0:raise ValueError('INVALID_RANGE_NOISE')
        self.Odometry, self.LaserScan, self.Transform = Odometry, LaserScan, TransformStamped
        self.odom = node.create_publisher(Odometry, 'navigation/odom', 5)
        self.scan = node.create_publisher(LaserScan, 'navigation/scan', 5)
        self.tf = TransformBroadcaster(node)
        self.static = StaticTransformBroadcaster(node)
        frames = robot['frames']
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = frames['base'], frames['laser']
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = robot['laser']['xyz']
        t.transform.rotation.z = math.sin(robot['laser']['yaw']/2)
        t.transform.rotation.w = math.cos(robot['laser']['yaw']/2)
        identity=TransformStamped(); identity.header.stamp=t.header.stamp
        identity.header.frame_id=frames['map']; identity.child_frame_id=frames['odom']
        identity.transform.rotation.w=1.
        self.static.sendTransform([t] if self.external else [t,identity])
        if not self.external:
            self.map = node.create_publisher(OccupancyGrid, 'navigation/map', QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            m = OccupancyGrid(); m.header.frame_id = frames['map']; m.header.stamp = t.header.stamp
            resolution=config['resolution']; xmin,xmax,ymin,ymax=config['bounds']
            m.info.resolution=resolution; m.info.width=round((xmax-xmin)/resolution); m.info.height=round((ymax-ymin)/resolution)
            m.info.origin.position.x=xmin; m.info.origin.position.y=ymin; m.info.origin.orientation.w=1.
            m.data=[100 if any(abs(xmin+(x+.5)*resolution-b['center'][0])<=b['size'][0]/2 and
                              abs(ymin+(y+.5)*resolution-b['center'][1])<=b['size'][1]/2
                              for b in config['boxes'] if b.get('mapped',True)) else 0
                    for y in range(m.info.height) for x in range(m.info.width)]
            self.map.publish(m)
        self.status=node.create_publisher(String,'navigation/scene_status',1)
        self.command=node.create_subscription(String,'navigation/scene_command',self.change,1)
        self.last_scan=0.
        self.pose_enabled=True;self.scan_enabled=True
        self.fault=node.create_subscription(String,'navigation/sensor_fault',
            self.set_fault,1)

    def set_fault(self, msg):
        self.pose_enabled=msg.data!='pose_loss'
        self.scan_enabled=msg.data!='scan_loss'

    def change(self, msg):
        from pxr import UsdGeom, Gf
        from std_msgs.msg import String
        if msg.data not in ('clear','detour','detour_lower','blocked'): return
        for b in self.cfg['boxes']:
            if b.get('variant'):
                variant='detour' if msg.data=='detour_lower' else msg.data
                center=list(b['center']) if b['variant']==variant else [b['center'][0],b['center'][1],-5.]
                if msg.data=='detour_lower' and b['variant']=='detour':center[1]=-abs(center[1])
                UsdGeom.Xformable(self.world.stage.GetPrimAtPath('/World/Room/'+b['name'])).GetOrderedXformOps()[0].Set(Gf.Vec3d(*center))
        self.status.publish(String(data=msg.data))

    def publish(self, feedback, odometry=None):
        from omni.physx import get_physx_scene_query_interface
        f=self.robot['frames']; laser=self.robot['laser']; p=feedback.base_pose
        # Ground truth remains local to ray generation and independent feedback.
        # External mode only publishes encoder odometry; algorithms own map->odom.
        msg=self.Odometry(); msg.header.stamp=feedback.stamp; msg.header.frame_id=f['odom']; msg.child_frame_id=f['base']
        if self.external:
            if odometry is None:raise ValueError('MISSING_ENCODER_ODOMETRY')
            x,y,a,v,w=odometry
            msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
            msg.pose.pose.orientation.z=math.sin(a/2);msg.pose.pose.orientation.w=math.cos(a/2)
            msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w
            msg.pose.covariance[0]=msg.pose.covariance[7]=.01;msg.pose.covariance[35]=.01
        else:
            msg.pose.pose=p; msg.twist.twist=feedback.base_velocity
        if self.pose_enabled: self.odom.publish(msg)
        t=self.Transform(); t.header=msg.header; t.child_frame_id=f['base']
        t.transform.translation.x=msg.pose.pose.position.x; t.transform.translation.y=msg.pose.pose.position.y
        q=p.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.external:t.transform.rotation=msg.pose.pose.orientation
        else:t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
        if self.pose_enabled: self.tf.sendTransform(t)
        now=feedback.stamp.sec+feedback.stamp.nanosec/1e9
        if not self.scan_enabled or now-self.last_scan<1/laser['rate']: return
        self.last_scan=now
        scan=self.LaserScan(); scan.header.stamp=feedback.stamp; scan.header.frame_id=f['laser']
        scan.angle_min=-math.pi; scan.angle_increment=2*math.pi/laser['beams']; scan.angle_max=scan.angle_min+(laser['beams']-1)*scan.angle_increment
        scan.range_min=laser['min_range']; scan.range_max=laser['max_range']; scan.scan_time=1/laser['rate']
        x,y,z=laser['xyz']; origin=(p.position.x+math.cos(yaw)*x-math.sin(yaw)*y,
            p.position.y+math.sin(yaw)*x+math.cos(yaw)*y,z)
        query=get_physx_scene_query_interface(); ranges=[]
        for i in range(laser['beams']):
            angle=yaw+laser['yaw']+scan.angle_min+i*scan.angle_increment
            hits=[]
            def hit(h):
                if not str(h.collision).startswith(self.excluded_path+'/'): hits.append(float(h.distance))
                return True
            query.raycast_all(origin,(math.cos(angle),math.sin(angle),0.),scan.range_max,hit)
            distance=min(hits,default=float('inf'))
            if math.isfinite(distance):distance+=self.random.gauss(0.,self.noise)
            ranges.append(distance if scan.range_min<=distance<=scan.range_max else float('inf'))
        scan.ranges=ranges; self.scan.publish(scan)
