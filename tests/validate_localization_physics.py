#!/usr/bin/env python3
"""SLAM -> saved map -> AMCL -> navigation in an already running Isaac fixture.

Owns only the localization launch. Requires navigation_physics.launch with
require_scan:=true and the mapping sites. Ground truth is recorded for scoring,
never published as a map, TF correction, navigation input or initial pose.
"""
import argparse
import gzip
import json
import math
import subprocess
import signal
import time
import traceback
import uuid
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_prefix
from bindu_interfaces.msg import SimulationFeedback, NavigationPose, RuntimeEvent, MotionCommand, ExecutionState
from bindu_interfaces.action import NavigateToSite
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from validate_ros import until, wait, terminate


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(t):return t.sec+t.nanosec/1e9

def main():
    p=argparse.ArgumentParser();p.add_argument('--namespace',default='/bindu_navigation')
    p.add_argument('--robot',type=Path,required=True);p.add_argument('--sites',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--map',type=Path)
    p.add_argument('--phase',choices=['full','mapping','localization','restore'],default='full')
    p.add_argument('--pose-graph',type=Path)
    args=p.parse_args()
    if args.phase=='restore' and not args.pose_graph:p.error('--phase restore requires --pose-graph')
    args.output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads(args.sites.read_text());sites={r['name']:r for r in cfg['sites']}
    ns=args.namespace;results=[];seen={};physical=[];poses=[];odom=[];estimates=[];statics=set();process=None
    rclpy.init();node=rclpy.create_node('localization_physics_validator_'+uuid.uuid4().hex[:8])
    raw=gzip.open(args.output/'observations.jsonl.gz','wt')
    def record(topic,msg):
        seen[topic]=msg
        raw.write(json.dumps({'topic':topic,'received':time.time(),'data':message_to_ordereddict(msg)})+'\n')
        if topic=='simulation/feedback':
            physical.append({'stamp':stamp(msg.stamp),'x':msg.base_pose.position.x,'y':msg.base_pose.position.y,
                'yaw':yaw(msg.base_pose.orientation),'v':msg.base_velocity.linear.x,'w':msg.base_velocity.angular.z})
        elif topic=='navigation/backend/pose':
            poses.append({'stamp':stamp(msg.pose.header.stamp),'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,'yaw':yaw(msg.pose.pose.orientation)})
        elif topic=='navigation/odom':
            odom.append({'stamp':stamp(msg.header.stamp),'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,'yaw':yaw(msg.pose.pose.orientation)})
        elif topic=='localization/pose':
            estimates.append({'stamp':stamp(msg.header.stamp),'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,'yaw':yaw(msg.pose.pose.orientation),
                'covariance':list(msg.pose.covariance)})
        elif topic=='tf_static':
            statics.update((t.header.frame_id,t.child_frame_id) for t in msg.transforms)
    best=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,typ,qos in [('simulation/feedback',SimulationFeedback,best),('navigation/backend/pose',NavigationPose,best),
                         ('navigation/odom',Odometry,100),('navigation/scan',LaserScan,best),('navigation/map',OccupancyGrid,latched),
                         ('localization/pose',PoseWithCovarianceStamped,best),('tf_static',TFMessage,latched),
                         ('navigation/backend/ready',Bool,1),('execution/state',ExecutionState,100),('events',RuntimeEvent,100)]:
        node.create_subscription(typ,ns+'/'+topic,lambda m,t=topic:record(t,m),qos)
    action=ActionClient(node,NavigateToSite,ns+'/navigation/navigate_to_site')
    sensor_fault=node.create_publisher(String,ns+'/navigation/sensor_fault',1)
    init_pose=node.create_publisher(PoseWithCovarianceStamped,ns+'/initialpose',1)
    def drain(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def save_result(row):
        results.append(row);print(json.dumps(row),flush=True)
        (args.output/'results.json').write_text(json.dumps(results,indent=2))
    def owner(topic):return sorted(i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(ns+'/'+topic))
    def stop_localizer():
        if process and process.poll() is None:
            # Deactivate/cleanup before invalidating the ROS context: SLAM's
            # teardown may still create timers while cleaning lifecycle state.
            from nav2_msgs.srv import ManageLifecycleNodes
            manager=node.create_client(ManageLifecycleNodes,ns+'/localization_manager/manage_nodes')
            try:
                if manager.wait_for_service(timeout_sec=2.):
                    response=wait(node,manager.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN)),seconds=8.)
                    if not response.success:print('LOCALIZATION_SHUTDOWN_REJECTED',flush=True)
            except Exception as exc:print('LOCALIZATION_SHUTDOWN_FAILED: '+str(exc),flush=True)
            finally:node.destroy_client(manager)
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8.)
            except subprocess.TimeoutExpired:terminate(process)

    def launch(mode,mapfile=None):
        nonlocal process
        if process:stop_localizer();process=None;drain(1.)
        seen.pop('navigation/map',None);seen.pop('localization/pose',None)
        cmd=['ros2','launch','bindu_runtime','localization.launch.py','namespace:='+ns,'navigation_robot:='+str(args.robot.resolve()),'mode:='+mode]
        if mode=='mapping' and args.pose_graph:cmd.append('pose_graph:='+str(args.pose_graph.resolve()))
        if mapfile:cmd.append('map:='+str(mapfile.resolve()))
        with (args.output/(mode+'.log')).open('w') as log:process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        until(node,lambda:'navigation/map' in seen,seconds=30.)
        expected=ns+('/slam_toolbox' if mode=='mapping' else '/map_server')
        reported=[None]
        def map_owner_ready():
            current=owner('navigation/map')
            if current!=reported[0]:
                print(json.dumps({'map_publishers':current,'expected':expected}),flush=True);reported[0]=current
            return current==[expected]
        until(node,map_owner_ready,seconds=25.)
        assert ('map','odom') not in statics,'ground truth map->odom must be disabled'
    def navigate(site,inject=False):
        until(node,lambda:seen.get('navigation/backend/ready') and seen['navigation/backend/ready'].data,seconds=35.)
        started=time.time();index=len(physical)
        goal=wait(node,action.send_goal_async(NavigateToSite.Goal(task_id='localization_'+uuid.uuid4().hex[:8],site=site)))
        assert goal.accepted,'navigation action rejected'
        future=goal.get_result_async();injected=None
        try:
            while not future.done() and time.time()-started<130:
                rclpy.spin_once(node,timeout_sec=.02)
                if inject and injected is None and physical and abs(physical[-1]['v'])>.05:
                    injected=physical[-1].copy();sensor_fault.publish(String(data='scan_loss'))
            assert future.done(),'navigation timeout'
            response=future.result();drain(.5);last=physical[-1]
            assert 0<=time.time()-last['stamp']<.2,'stale physical feedback'
            target=sites[site];error=math.hypot(last['x']-target['x'],last['y']-target['y'])
            yaw_error=abs(math.atan2(math.sin(last['yaw']-target['yaw']),math.cos(last['yaw']-target['yaw'])))
            row={'case':'scan_loss' if inject else 'navigate','site':site,'start_stamp':started,'end_stamp':last['stamp'],
                'success':response.result.success,'status':response.status,'code':response.result.code,'physical_position_error_m':error,'physical_yaw_error_rad':yaw_error,
                'final_v':last['v'],'final_w':last['w'],'physical_samples':len(physical)-index}
            if inject:
                assert injected,'fault must be injected during actual forward motion'
                row['injection_stamp']=injected['stamp'];row['injection_velocity']=injected['v']
                row['distance_after_loss_m']=math.hypot(last['x']-injected['x'],last['y']-injected['y'])
                row['scan_age_at_result_s']=time.time()-stamp(seen['navigation/scan'].header.stamp)
                # RPP TF_ERROR (102) may abort before the public pose-age gate.
                # Both are localization failures; arbitrary Action failures are not accepted.
                assert not response.result.success and response.result.code in ('NAV_POSE_STALE','NAV_ACTION_FAILED:102'),row
                assert row['scan_age_at_result_s']>.3 and row['distance_after_loss_m']<.15,row
            else:assert response.result.success and error<.25 and yaw_error<.14,row
            assert abs(last['v'])<.01 and abs(last['w'])<.01,row
            row['passed']=True;return row
        finally:sensor_fault.publish(String(data='clear'));drain(1.)
    def map_drive(site):
        # A bounded synthetic teleoperation route collects scans before a full
        # map exists. It uses SLAM/odometry feedback, never simulator truth.
        until(node,lambda:'navigation/backend/pose' in seen and 'execution/state' in seen,seconds=20.)
        leases=node.create_client(Lease,ns+'/execution/lease')
        submit=node.create_client(SubmitMotion,ns+'/execution/submit')
        control=node.create_client(ControlExecution,ns+'/execution/control')
        lease=wait(node,leases.call_async(Lease.Request(operation='acquire',owner='mapping-test',resources=['base'])))
        assert lease.ok,lease
        acquired=time.time()
        until(node,lambda:stamp(seen['execution/state'].stamp)>acquired)
        renewal=node.create_timer(.5,lambda:leases.call_async(Lease.Request(operation='renew',lease_id=lease.lease_id,epoch=lease.epoch)))
        target=sites[site];start=time.time();revision=seen['execution/state'].revision;position_done=False
        try:
            while time.time()-start<80.:
                pose=seen['navigation/backend/pose'].pose
                assert 0<=time.time()-stamp(pose.header.stamp)<.3,'mapping pose stale'
                assert 0<=time.time()-stamp(seen['navigation/scan'].header.stamp)<.3,'mapping scan stale'
                dx=target['x']-pose.pose.position.x;dy=target['y']-pose.pose.position.y
                distance=math.hypot(dx,dy);heading=yaw(pose.pose.orientation)
                position_done=position_done or distance<=.06
                desired=target['yaw'] if position_done else math.atan2(dy,dx)
                angle=math.atan2(math.sin(desired-heading),math.cos(desired-heading))
                if position_done and abs(angle)<.06:break
                linear=min(.2,.6*distance) if not position_done and abs(angle)<.25 else 0.
                angular=max(-.35,min(.35,1.2*angle))
                cmd=MotionCommand(schema_version=1,command_id=uuid.uuid4().hex,task_id='mapping_'+site,
                    lease_id=lease.lease_id,epoch=lease.epoch,profile_hash=seen['simulation/feedback'].profile_hash,
                    mode='base_velocity',resource_group='base',stamp=node.get_clock().now().to_msg(),
                    valid_for=.4,duration=.3,expected_revision=revision)
                cmd.velocity.linear.x=linear;cmd.velocity.angular.z=angular
                reply=wait(node,submit.call_async(SubmitMotion.Request(command=cmd)))
                assert reply.accepted,reply;revision=reply.revision;drain(.045)
            else:raise AssertionError('mapping waypoint timeout')
        finally:
            reply=wait(node,control.call_async(ControlExecution.Request(operation='stop',lease_id=lease.lease_id,epoch=lease.epoch)))
            assert reply.ok,reply
            until(node,lambda:seen['execution/state'].state!='STOPPING' and not seen['execution/state'].stop_failures,seconds=6.)
            reply=wait(node,leases.call_async(Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch)))
            node.destroy_timer(renewal);assert reply.ok,reply
            drain(.5)
        last=physical[-1];error=math.hypot(last['x']-target['x'],last['y']-target['y'])
        assert error<.25,error
        return {'case':'mapping_waypoint','site':site,'start_stamp':start,'end_stamp':last['stamp'],
                'physical_position_error_m':error,'passed':True}
    try:
        assert action.wait_for_server(timeout_sec=20.)
        until(node,lambda:all(t in seen for t in ('simulation/feedback','navigation/odom','navigation/scan')),seconds=25.)
        drain(2.)
        assert not node.get_publishers_info_by_topic(ns+'/navigation/map'),'simulator must not publish a synthetic map'
        mapfile=(args.map.resolve() if args.map else (args.output/'room.yaml').resolve())
        if args.phase in ('full','mapping','restore'):
            launch('mapping')
            m=seen['navigation/map'];initial_known=sum(v>=0 for v in m.data)
            save_result({'case':'slam_started','map_publishers':owner('navigation/map'),'initial_known_cells':initial_known,'passed':True})
            if args.phase!='restore':
                for site in ['mapping_south','mapping_east','mapping_north','mapping_west','home']:
                    save_result(map_drive(site))
            drain(2.);m=seen['navigation/map'];occupied=sum(v>=65 for v in m.data);free=sum(0<=v<25 for v in m.data)
            assert occupied>100 and free>1000,(occupied,free)
            (args.output/'slam_grid.json').write_text(json.dumps(message_to_ordereddict(m)))
            from slam_toolbox.srv import SerializePoseGraph
            client=node.create_client(SerializePoseGraph,ns+'/slam_toolbox/serialize_map')
            assert client.wait_for_service(timeout_sec=5.)
            reply=wait(node,client.call_async(SerializePoseGraph.Request(filename=str((args.output/'room_graph').resolve()))),seconds=10.)
            assert reply.result==0,reply
            binary=Path(get_package_prefix('nav2_map_server'))/'lib/nav2_map_server/map_saver_cli'
            with (args.output/'map-save.log').open('w') as log:
                saver=subprocess.Popen([str(binary),'-f',str(mapfile.with_suffix('')),'--ros-args','-r','__ns:='+ns,'-r','map:=navigation/map','-p','save_map_timeout:=10.0','-p','map_subscribe_transient_local:=true'],stdout=log,stderr=subprocess.STDOUT)
                until(node,lambda:saver.poll() is not None,seconds=15.)
                assert saver.returncode==0,(args.output/'map-save.log').read_text()
            assert mapfile.is_file() and mapfile.with_suffix('.pgm').is_file(),'map save missing'
            save_result({'case':'map_saved','yaml':str(mapfile),'occupied_cells':occupied,'free_cells':free,'width':m.info.width,'height':m.info.height,
                         'resolution':m.info.resolution,'graph_saved':True,'passed':True})
        if args.phase in ('full','localization','restore'):
            launch('localization',mapfile)
            until(node,lambda:init_pose.get_subscription_count()>0,seconds=10.)
            # Explicit approximate operator prior, not simulator ground truth.
            initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=node.get_clock().now().to_msg()
            initial.pose.pose.position.x=.25;initial.pose.pose.position.y=-.2
            initial.pose.pose.orientation.z=math.sin(.12/2);initial.pose.pose.orientation.w=math.cos(.12/2)
            initial.pose.covariance[0]=initial.pose.covariance[7]=.25**2;initial.pose.covariance[35]=.2**2
            init_pose.publish(initial)
            nomotion=node.create_client(Empty,ns+'/request_nomotion_update');assert nomotion.wait_for_service(timeout_sec=10.)
            before=len(estimates)
            wait(node,nomotion.call_async(Empty.Request()))
            until(node,lambda:len(estimates)>before,seconds=15.)
            before=len(estimates)
            for _ in range(15):wait(node,nomotion.call_async(Empty.Request()));drain(.2)
            until(node,lambda:len(estimates)>=before+5,seconds=10.)
            estimate=estimates[-1];last=physical[-1];error=math.hypot(estimate['x']-last['x'],estimate['y']-last['y'])
            angle=abs(math.atan2(math.sin(estimate['yaw']-last['yaw']),math.cos(estimate['yaw']-last['yaw'])))
            assert error<.12 and angle<.12,(estimate,last,error,angle)
            save_result({'case':'amcl_reloaded_localization','map_publishers':owner('navigation/map'),
                'prior':[.25,-.2,.12],'estimate':estimate,'physical_error_m':error,'yaw_error_rad':angle,'passed':True})
            save_result(navigate('pickup'));save_result(navigate('home'))
            save_result(navigate('pickup',inject=True))
            save_result(navigate('home'))
    except Exception as exc:
        save_result({'case':'failure','passed':False,'error':str(exc),'traceback':traceback.format_exc()})
    finally:
        sensor_fault.publish(String(data='clear'));drain(.2)
        if process:stop_localizer()
        (args.output/'trajectory.json').write_text(json.dumps({'physical':physical,'global':poses,'odometry':odom,'localization':estimates}))
        (args.output/'results.json').write_text(json.dumps(results,indent=2))
        raw.close();node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if results and all(r['passed'] for r in results) else 1)

if __name__=='__main__':main()
