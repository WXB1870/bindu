"""Model-independent Nav2 process/session adapter for the simulation runtime.

Each NavigateToPose goal owns fresh planner/controller/navigator processes and a
unique cmd_vel topic. Publisher GIDs are pinned before dispatch. Late commands
retain their original identity; no shared Twist topic is relabeled with a current
UUID. This conservative first integration trades startup time for isolation.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from bindu_interfaces.msg import NavigationVelocity, NavigationPose, ExecutionState
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_prefix
from bindu_navigation.navigation import NavigationConfig, SessionVelocityFence
from .common import seconds, stamp, spin


def parameters(robot, ns, session, bt_file):
    """Nav2 algorithm baseline, with all embodiment values supplied by config."""
    frames=robot['frames']
    def costmap(global_map):
        cfg={'global_frame':frames['map'] if global_map else frames['odom'],
            'robot_base_frame':frames['base'], 'update_frequency':5., 'publish_frequency':2.,
            'transform_tolerance':.2, 'resolution':.05, 'footprint':json.dumps(robot['footprint']),
            'footprint_padding':.03, 'always_send_full_costmap':True,
            'plugins':['static_layer','obstacle_layer','inflation_layer'],
            'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_topic':ns+'/navigation/map','map_subscribe_transient_local':True},
            'obstacle_layer':{'plugin':'nav2_costmap_2d::ObstacleLayer','observation_sources':'laser',
                'laser':{'topic':ns+'/navigation/scan','data_type':'LaserScan','clearing':True,'marking':True,
                    'max_obstacle_height':2.,'raytrace_max_range':10.,'obstacle_max_range':9.,'inf_is_valid':True}},
            'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.8,'cost_scaling_factor':3.}}
        if global_map:
            # NavFn plans center positions. A circumscribed global envelope
            # preserves clearance for every orientation of a rectangular body.
            radius=max(math.hypot(x,y) for x,y in robot['footprint'])+.03
            cfg['footprint']=json.dumps([[radius*math.cos(i*math.pi/16),radius*math.sin(i*math.pi/16)] for i in range(32)])
            cfg['footprint_padding']=0.
        if not global_map: cfg.update(rolling_window=True,width=5,height=5)
        else: cfg['track_unknown_space']=True
        return {'ros__parameters':cfg}
    v,w=robot['linear_velocity'],robot['angular_velocity']
    aw=robot['angular_acceleration']
    return {
        session+'/planner_server':{'ros__parameters':{'expected_planner_frequency':1.,'planner_plugins':['GridBased'],
            'GridBased':{'plugin':'nav2_navfn_planner::NavfnPlanner','tolerance':.1,'use_astar':False,'allow_unknown':False}}},
        session+'/global_costmap/global_costmap':costmap(True),
        session+'/local_costmap/local_costmap':costmap(False),
        session+'/controller_server':{'ros__parameters':{
            'controller_frequency':20., 'enable_stamped_cmd_vel':True, 'odom_topic':ns+'/navigation/odom',
            'min_x_velocity_threshold':.001,'min_y_velocity_threshold':.001,'min_theta_velocity_threshold':.001,
            'failure_tolerance':1.,'progress_checker_plugins':['progress_checker'],'goal_checker_plugins':['goal_checker'],
            'controller_plugins':['FollowPath'],
            'progress_checker':{'plugin':'nav2_controller::SimpleProgressChecker','required_movement_radius':.1,'movement_time_allowance':15.},
            'goal_checker':{'plugin':'nav2_controller::SimpleGoalChecker', 'xy_goal_tolerance':robot['position_tolerance'],
                'yaw_goal_tolerance':robot['yaw_tolerance'],'stateful':True},
            'FollowPath':{'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'desired_linear_vel':v,'lookahead_dist':.55,'min_lookahead_dist':.3,'max_lookahead_dist':.7,
                'use_velocity_scaled_lookahead_dist':False,'transform_tolerance':.2,
                'rotate_to_heading_angular_vel':w,'max_angular_accel':aw,'use_rotate_to_heading':True,
                'rotate_to_heading_min_angle':.5,'allow_reversing':False,
                'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':1.,
                'use_regulated_linear_velocity_scaling':True,'use_cost_regulated_linear_velocity_scaling':True,
                'regulated_linear_scaling_min_speed':.08,'min_approach_linear_velocity':.04,
                'approach_velocity_scaling_dist':.6,'cost_scaling_dist':.5,'cost_scaling_gain':1.,
                'inflation_cost_scaling_factor':3.}}},
        session+'/bt_navigator':{'ros__parameters':{'global_frame':frames['map'],'robot_base_frame':frames['base'],
            'odom_topic':ns+'/navigation/odom','bt_loop_duration':20,'default_server_timeout':1000,
            'wait_for_service_timeout':5000,'navigators':['navigate_to_pose'],
            'navigate_to_pose':{'plugin':'nav2_bt_navigator::NavigateToPoseNavigator'},'default_nav_to_pose_bt_xml':str(bt_file)}},
        session+'/lifecycle_manager':{'ros__parameters':{'autostart':True,'bond_timeout':2.,
            'node_names':['controller_server','planner_server','bt_navigator']}}}


BT='''<root BTCPP_format="4" main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><PipelineSequence name="NavigateWithReplanning">
<RateController hz="1.0"><Fallback><IsPathValid path="{path}"/><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" error_code_id="{compute_path_error_code}"/></Fallback></RateController>
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker" error_code_id="{follow_path_error_code}"/>
</PipelineSequence></BehaviorTree></root>'''


class PreparedSession:
    """One prewarmed process set, consumed by exactly one parent goal."""
    def __init__(self, node):
        import yaml
        self.node=node; self.token=uuid.uuid4().hex
        self.ns=node.get_namespace().rstrip('/')+'/nav2_'+self.token
        self.folder=node.output/self.token;self.folder.mkdir()
        self.processes=[];self.logs=[];self.fence=None;self.fault='';self.count=0
        self.first_command=None;self.ready=False;self.activated_at=None;self.native_started=False
        self.created=time.monotonic();self.stable_since=None
        self.events=(self.folder/'commands.jsonl').open('w')
        self.paths=(self.folder/'plans.jsonl').open('w')
        from nav_msgs.msg import Path as NavPath
        self.path_sub=node.create_subscription(NavPath,self.ns+'/plan',self.record_path,1,callback_group=node.group)
        bt=self.folder/'navigate.xml';bt.write_text(BT)
        config=self.folder/'params.yaml'
        config.write_text(yaml.safe_dump(parameters(node.robot,node.get_namespace(),self.ns,bt)))
        from std_msgs.msg import String
        from lifecycle_msgs.srv import GetState
        from rcl_interfaces.srv import SetParameters
        self.GetState=GetState
        self.binder=node.create_client(SetParameters,self.ns+'/velocity_identity/set_parameters',callback_group=node.group)
        self.sub=node.create_subscription(NavigationVelocity,self.ns+'/identified_velocity',self.receive,10,callback_group=node.group)
        self.fault_sub=node.create_subscription(String,self.ns+'/identity_fault',lambda m:setattr(self,'fault',m.data),1,callback_group=node.group)
        self.client=ActionClient(node,NavigateToPose,self.ns+'/navigate_to_pose',callback_group=node.group)
        self.life=node.create_client(GetState,self.ns+'/bt_navigator/get_state',callback_group=node.group);self.life_future=None;self.life_requested=0.
        for package,exe in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                            ('nav2_bt_navigator','bt_navigator'),('nav2_lifecycle_manager','lifecycle_manager')]:
            binary=Path(get_package_prefix(package))/'lib'/package/exe
            cmd=[str(binary),'--ros-args','-r','__ns:='+self.ns,'--params-file',str(config),
                 '-r','/tf:='+node.get_namespace()+'/tf','-r','/tf_static:='+node.get_namespace()+'/tf_static']
            if exe=='lifecycle_manager':cmd+=['-r','__node:=lifecycle_manager']
            self.start(exe,cmd)

    def record_path(self, msg):
        self.paths.write(json.dumps({'stamp':seconds(msg.header.stamp),
            'points':[[p.pose.position.x,p.pose.position.y] for p in msg.poses]})+'\n')
        self.paths.flush()

    def start(self,name,cmd):
        log=(self.folder/(name+'.log')).open('w');self.logs.append(log)
        self.processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))

    def poll(self):
        n=self.node
        if any(p.poll() is not None for p in self.processes):self.fault='NAV2_PROCESS_EXITED'
        if self.fault:return
        if self.life_future is not None and not self.life_future.done() and time.monotonic()-self.life_requested>1.:
            self.life.remove_pending_request(self.life_future);self.life_future.cancel();self.life_future=None
        if self.life_future is None and self.life.service_is_ready():
            self.life_future=self.life.call_async(self.GetState.Request());self.life_requested=time.monotonic()
        if self.life_future is not None and self.life_future.done():
            if self.life_future.result().current_state.id==3 and self.activated_at is None:self.activated_at=n.now()
            self.life_future=None
        pubs=n.get_publishers_info_by_topic(self.ns+'/cmd_vel')
        if not self.native_started and self.activated_at and len(pubs)==1 and pubs[0].node_name=='controller_server' and pubs[0].node_namespace==self.ns:
            self.gid=bytes(pubs[0].endpoint_gid)
            self.start('velocity_identity',[str(n.identity_bridge),'',self.ns,self.gid.hex(),str(n.now()),
                '--ros-args','-r','__ns:='+self.ns])
            self.native_started=True
        inputs_ready=(self.native_started and self.client.server_is_ready() and n.transport_ready() and n.pose_fresh()
            and n.count_publishers(self.ns+'/identified_velocity')==1 and self.binder.service_is_ready())
        if not inputs_ready:self.stable_since=None
        elif self.stable_since is None:self.stable_since=n.now()
        self.ready=bool(inputs_ready and n.now()-self.stable_since>2.)

    def receive(self, identified):
        fence=self.fence
        if fence is None or fence.closed:return
        if identified.goal_id!=fence.goal_id or identified.source_id!=self.ns:
            self.fault='NAV2_IDENTITY_MISMATCH';return
        msg=identified.command
        code=fence.accept(self.gid,seconds(msg.header.stamp),self.node.now(),self.node.config.input_timeout)
        if code:self.fault=code;return
        command=copy.deepcopy(msg);command.header.frame_id=self.node.config.base_frame
        scale=min(1.,self.node.robot['linear_velocity']/max(abs(command.twist.linear.x),1e-9),
            self.node.robot['angular_velocity']/max(abs(command.twist.angular.z),1e-9))
        command.twist.linear.x*=scale;command.twist.angular.z*=scale
        self.node.send(fence,command);self.count+=1;self.first_command=self.first_command or self.node.now()
        self.events.write(json.dumps({'stamp':seconds(msg.header.stamp),'v':msg.twist.linear.x,'w':msg.twist.angular.z,
            'bounded_v':command.twist.linear.x,'bounded_w':command.twist.angular.z,'scale':scale,
            'publisher_gid':self.gid.hex(),'goal':fence.goal_id,'source':self.ns})+'\n')

    def retire(self):
        if self.fence:self.fence.close()
        n=self.node
        n.destroy_subscription(self.path_sub);n.destroy_subscription(self.sub);n.destroy_subscription(self.fault_sub)
        self.client.destroy();n.destroy_client(self.life);n.destroy_client(self.binder)
        for p in self.processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        def reap():
            for p in self.processes:
                try:p.wait(timeout=2.)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=2.)
            self.events.close();self.paths.close()
            for log in self.logs:log.close()
            (self.folder/'result.json').write_text(json.dumps({'fault':self.fault,'accepted':self.count,
                'first_command':self.first_command,'seconds':time.monotonic()-self.created,
                'exits':[p.returncode for p in self.processes]},indent=2))
        thread=threading.Thread(target=reap);thread.start();n.reapers.append(thread)


class Nav2SessionNode(Node):
    def __init__(self):
        super().__init__('nav2_session')
        for name,default in [('navigation_robot',''),('navigation_config',''),('identity_bridge',''),('output','/tmp/bindu-nav2-sessions')]:
            self.declare_parameter(name,default)
        self.robot=json.loads(Path(self.get_parameter('navigation_robot').value).read_text())
        self.config=NavigationConfig.load(self.get_parameter('navigation_config').value)
        if self.robot['frames']['map']!=self.config.frame or self.robot['frames']['base']!=self.config.base_frame:
            raise ValueError('NAVIGATION_FRAME_CONFIG_MISMATCH')
        self.identity_bridge=Path(self.get_parameter('identity_bridge').value)
        if not self.identity_bridge.is_file():raise ValueError('BUILD_NAV2_IDENTITY_BRIDGE_FIRST')
        self.output=Path(self.get_parameter('output').value);self.output.mkdir(parents=True,exist_ok=True)
        self.declare_parameter('require_scan',False)
        self.require_scan=self.get_parameter('require_scan').value;self.scan_stamp=None
        self.group=ReentrantCallbackGroup();self.posture=None;self.busy=False;self.session=None;self.reapers=[];self.prepare_after=0.
        self.create_subscription(ExecutionState,'execution/state',lambda m:setattr(self,'posture',m),1,callback_group=self.group)
        if self.require_scan:
            from sensor_msgs.msg import LaserScan
            self.create_subscription(LaserScan,'navigation/scan',lambda m:setattr(self,'scan_stamp',seconds(m.header.stamp)),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=self.group)
        self.velocity=self.create_publisher(NavigationVelocity,'navigation/backend/velocity',QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pose=self.create_publisher(NavigationPose,'navigation/backend/pose',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        from std_msgs.msg import Bool
        self.Bool=Bool;self.readiness=self.create_publisher(Bool,'navigation/backend/ready',1)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_timer(.025,self.publish_pose,callback_group=self.group)
        self.create_timer(.1,self.prepare,callback_group=self.group)
        self.server=ActionServer(self,NavigateToPose,'navigation/backend/navigate_to_pose',execute_callback=self.execute,
            goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)

    def prepare(self):
        self.reapers=[t for t in self.reapers if t.is_alive()]
        if not self.busy:
            if self.session is None and self.now()>=self.prepare_after and self.transport_ready():self.session=PreparedSession(self)
            if self.session:self.session.poll()
        self.readiness.publish(self.Bool(data=bool(not self.busy and self.session and self.session.ready and not self.session.fault)))

    def goal(self, request):
        if (self.busy or not self.session or not self.session.ready or self.session.fault or
            request.pose.header.frame_id!=self.config.frame or not self.transport_ready()):return GoalResponse.REJECT
        self.busy=True;return GoalResponse.ACCEPT

    def transport_ready(self, moving=False):
        state=self.posture
        if not state or not 0<=self.now()-seconds(state.joints.header.stamp)<.2:return False
        values=dict(zip(state.joints.name,state.joints.position));speeds=dict(zip(state.joints.name,state.joints.velocity))
        return all(n in values and n in speeds and abs(values[n]-q)<=self.robot['transport_position_tolerance']
                   and abs(speeds[n])<=self.robot['transport_motion_speed_tolerance' if moving else 'transport_speed_tolerance']
                   for n,q in self.robot.get('transport_positions',{}).items())

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def scan_fresh(self):
        return not self.require_scan or (self.scan_stamp is not None and 0<=self.now()-self.scan_stamp<self.config.pose_timeout)

    def pose_fresh(self):
        if not self.scan_fresh():return False
        try:
            t=self.buffer.lookup_transform(self.config.frame,self.config.base_frame,rclpy.time.Time())
            return 0<=self.now()-seconds(t.header.stamp)<self.config.pose_timeout
        except Exception:return False

    def publish_pose(self):
        if not self.scan_fresh():return
        try:
            t=self.buffer.lookup_transform(self.config.frame,self.config.base_frame,rclpy.time.Time())
            if not 0<=self.now()-seconds(t.header.stamp)<self.config.pose_timeout:return
            p=NavigationPose(map_id=self.config.map_id);p.pose.header=t.header
            p.pose.pose.position.x=t.transform.translation.x;p.pose.pose.position.y=t.transform.translation.y
            p.pose.pose.position.z=t.transform.translation.z;p.pose.pose.orientation=t.transform.rotation
            self.pose.publish(p)
        except Exception:return

    async def pause(self):
        f=Future(executor=self.executor)
        timer=self.create_timer(.02,lambda:f.set_result(None) if not f.done() else None,callback_group=self.group)
        try:await f
        finally:self.destroy_timer(timer)

    def send(self, fence, command):
        self.velocity.publish(NavigationVelocity(goal_id=fence.goal_id,source_id=fence.source_id,
            sequence=fence.next_sequence(),command=command))

    async def execute(self, goal):
        result=NavigateToPose.Result();session=self.session;child=None
        identity=bytes(goal.goal_id.uuid).hex();fence=SessionVelocityFence(identity,session.ns,self.now())
        fence.bind_publishers({session.gid});session.fence=fence
        (session.folder/'session.json').write_text(json.dumps({'goal_id':identity,'source_id':session.ns,
            'pids':[p.pid for p in session.processes],'robot_config':self.robot,'simulation_only':True,
            'prewarmed':True,'native_goal_binding':'one-time SetParameters before child dispatch'},indent=2))
        started=self.now();last_zero=0.
        try:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue
            binding=session.binder.call_async(SetParameters.Request(parameters=[Parameter(name='goal_id',
                value=ParameterValue(type=4,string_value=identity))]))
            dispatch=None;future=None
            while rclpy.ok():
                if goal.is_cancel_requested:
                    fence.close()
                    if child:child.cancel_goal_async()
                    goal.canceled();return result
                if any(p.poll() is not None for p in session.processes):raise RuntimeError('NAV2_PROCESS_EXITED')
                if session.fault:raise RuntimeError(session.fault)
                if not self.transport_ready(moving=True):raise RuntimeError('NAV_TRANSPORT_POSTURE_LOST')
                if self.now()-started>self.config.timeout-1:raise RuntimeError('NAV2_SESSION_TIMEOUT')
                if dispatch is None and binding.done():
                    if not all(r.successful for r in binding.result().results):raise RuntimeError('NAV2_GOAL_BINDING_FAILED')
                    dispatch=session.client.send_goal_async(goal.request)
                if child is None and dispatch is not None and dispatch.done():
                    child=dispatch.result()
                    if not child.accepted:raise RuntimeError('NAV2_CHILD_REJECTED')
                    future=child.get_result_async()
                if future is not None and future.done():
                    response=future.result();result=response.result;fence.close()
                    if response.status==4 and not result.error_code:goal.succeed()
                    else:goal.abort()
                    return result
                if session.first_command is None:
                    if self.now()-started>2.:raise RuntimeError('NAV2_FIRST_COMMAND_TIMEOUT')
                    if self.now()-last_zero>.05:
                        zero=TwistStamped();zero.header.stamp=self.get_clock().now().to_msg();zero.header.frame_id=self.config.base_frame
                        self.send(fence,zero);last_zero=self.now()
                elif self.now()-fence.last_stamp>self.config.input_timeout:raise RuntimeError('NAV2_COMMAND_TIMEOUT')
                await self.pause()
        except Exception as exc:
            session.fault=str(exc);result.error_code=900;result.error_msg=str(exc);fence.close()
            if child:child.cancel_goal_async()
            goal.abort();return result
        finally:
            session.retire();self.session=None;self.busy=False;self.prepare_after=self.now()+2.

    def destroy_node(self):
        if self.session:self.session.retire();self.session=None
        for thread in self.reapers:thread.join(timeout=10.)
        return super().destroy_node()


def main():spin(Nav2SessionNode,threaded=False)
if __name__=='__main__':main()
