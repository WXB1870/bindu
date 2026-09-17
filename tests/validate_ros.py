#!/usr/bin/env python3
"""Launch only owned simulation processes and verify ROS contracts end to end."""
import argparse
import hashlib
import platform
import sys
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import uuid
import rclpy
from rclpy.action import ActionClient
from bindu_interfaces.action import FetchDrink
from bindu_interfaces.msg import ExecutionState, RecorderHealth
from bindu_interfaces.srv import InjectFault, Lease, SubmitMotion, ControlExecution
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_interfaces.msg import MotionCommand
from builtin_interfaces.msg import Time


def wait(node, future, seconds=10):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
    if not future.done(): raise AssertionError('client future timed out')
    return future.result()


def until(node, predicate, seconds=10):
    end=time.monotonic()+seconds
    while not predicate() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
    assert predicate(), 'condition timed out'


def terminate(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL)
            p.wait(timeout=3)


def scenario(root, output, case, strategy='planner', profile='wheel_sim', fault='', cancel=False, launch=False):
    run=case+'_'+uuid.uuid4().hex[:8]
    ns='/bindu_test_'+uuid.uuid4().hex[:8]
    logs=output/'processes'/run
    logs.mkdir(parents=True)
    processes={}
    handles=[]
    node=rclpy.create_node('verifier_'+uuid.uuid4().hex[:8])
    seen={'state':None,'health':None,'stages':[]}
    subs=[node.create_subscription(ExecutionState,ns+'/execution/state',lambda m:seen.update(state=m),10),
          node.create_subscription(RecorderHealth,ns+'/recorder/health',lambda m:seen.update(health=m),5)]
    result=None
    try:
        if launch:
            log=(logs/'launch.log').open('w'); handles.append(log)
            processes['task']=subprocess.Popen(['ros2','launch','bindu_runtime','skeleton.launch.py',
                'namespace:='+ns, 'run_id:='+run,'output:='+str(output/'episodes')],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        for name in (() if launch else ('execution','perception','recorder','task')):
            log=(logs/(name+'.log')).open('w'); handles.append(log)
            cmd=['ros2','run','bindu_runtime',name,'--ros-args','-r','__ns:='+ns,
                 '-p','profile:='+str(root/'src/integration/bindu_runtime/config'/f'{profile}.json'),
                 '-p','run_id:='+run,'-p','output:='+str(output/'episodes')]
            processes[name]=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        action=ActionClient(node,FetchDrink,ns+'/tasks/fetch_drink')
        assert action.wait_for_server(timeout_sec=15),'no task server'
        until(node,lambda: seen['state'] and seen['health'] and seen['health'].ready)
        time.sleep(.4)  # Task process must discover its separate capability services.
        if fault in ('delay','stale','reject','hand:reject'):
            path='/perception/sim_fault' if fault in ('delay','stale') else '/execution/sim_fault'
            client=node.create_client(InjectFault,ns+path)
            assert client.wait_for_service(timeout_sec=5)
            assert wait(node,client.call_async(InjectFault.Request(fault=fault,value=1.2 if fault=='delay' else 0.))).ok
        goal=wait(node,action.send_goal_async(FetchDrink.Goal(task_id=run,object_id='drink',strategy=strategy),
                   feedback_callback=lambda m:seen['stages'].append(m.feedback.stage)))
        assert goal.accepted,'task rejected'
        future=goal.get_result_async()
        if cancel or fault in ('feedback_loss','hand:feedback_loss','recorder_loss','perception_loss'):
            until(node,lambda:'APPROACH' in seen['stages'] if cancel or fault in ('feedback_loss','hand:feedback_loss') else 'NAVIGATE' in seen['stages'])
            if cancel:
                canceled=wait(node,goal.cancel_goal_async())
                assert canceled.goals_canceling,'cancel rejected'
            elif fault in ('feedback_loss','hand:feedback_loss'):
                c=node.create_client(InjectFault,ns+'/execution/sim_fault')
                assert c.wait_for_service(timeout_sec=5)
                assert wait(node,c.call_async(InjectFault.Request(fault=fault))).ok
            else:
                name='recorder' if fault=='recorder_loss' else 'perception'
                os.killpg(processes[name].pid,signal.SIGKILL)
                processes[name].wait(timeout=3)
        response=wait(node,future,15)
        result={'case':case,'run_id':run,'profile':profile,'strategy':strategy,'status':response.status,
                'success':response.result.success,'code':response.result.code,'simulated':response.result.simulated,
                'stages':seen['stages']}
        assert response.result.simulated
        if cancel:
            assert response.status==5 and response.result.code=='CANCELED',result
            assert not response.result.success
        elif fault:
            assert response.status==6 and not response.result.success,result
            expected={'delay':'CAPABILITY_TIMEOUT','stale':'INVALID_OBSERVATION','reject':'DEVICE_COMMAND_REJECTED',
                      'feedback_loss':'FEEDBACK_STALE','recorder_loss':'RECORDER_UNAVAILABLE',
                      'perception_loss':'CAPABILITY_TIMEOUT','hand:reject':'DEVICE_COMMAND_REJECTED',
                      'hand:feedback_loss':'FEEDBACK_STALE'}[fault]
            if fault in ('feedback_loss','hand:feedback_loss'):
                assert any(x in response.result.code for x in (expected,'LEASE_RENEW_FAILED')),result
                assert 'STOP_UNCONFIRMED' in response.result.code,result
            else: assert expected in response.result.code,result
            if fault in ('delay','stale','perception_loss'): assert 'APPROACH' not in seen['stages'],result
        else:
            assert response.status==4 and response.result.success and response.result.held_object=='drink',result
            assert seen['stages']==['NAVIGATE','LOCATE','APPROACH','GRASP','RETURN'],result
            assert len(seen['state'].joints.name)==(4 if profile=='wheel_sim' else 6)
            assert abs(seen['state'].base_x)<.035,result
        # Confirm fresh stopping; feedback-loss must remain explicitly unconfirmed.
        if fault not in ('feedback_loss','hand:feedback_loss'):
            until(node,lambda:abs(seen['state'].base_velocity.linear.x)<1e-6)
        time.sleep(.2)
    finally:
        # Keep the recorder alive until producer shutdowns have been observed.
        for name in ('task','perception','execution','recorder'):
            if name in processes: terminate(processes[name])
        node.destroy_node()
        for h in handles: h.close()
    episode=output/'episodes'/run
    if fault!='recorder_loss':
        summary=json.loads((episode/'summary.json').read_text())
        assert summary['writer_complete'] and summary['dropped']==0,summary
        records=[json.loads(line) for line in (episode/'episode.jsonl').read_text().splitlines()]
        kinds={r['kind'] for r in records}
        assert {'event','feedback','reference'}<=kinds,kinds
        assert any(r['kind']=='event' and r['data']['state'].startswith('TASK_') for r in records)
        if result['success']:
            assert 'observation' in kinds
            refs=[r['data'] for r in records if r['kind']=='reference' and r['data']['mode']=='finite_trajectory']
            assert len(refs)==2
            assert all(r['observation_id'] for r in refs)
            if strategy=='chunk':
                assert all(len(r['points'])==4 and all(p['velocities'] and p['accelerations'] for p in r['points']) for r in refs)
        result['records']=len(records)
        result['recorder']=summary
    else:
        assert not (episode/'summary.json').exists(),'crash must not be marked complete'
        result['recorder']={'complete':False,'reason':'injected process loss'}
    result['passed']=True
    return result


def online_targets(root, output):
    """Exercise the online port at nominal 100 Hz and confirm command expiry."""
    ns='/bindu_stream_'+uuid.uuid4().hex[:8]
    run='online_'+uuid.uuid4().hex[:8]
    node=rclpy.create_node('stream_verifier_'+uuid.uuid4().hex[:8])
    logdir=output/'processes'/run
    logdir.mkdir(parents=True)
    log=(logdir/'execution.log').open('w')
    process=subprocess.Popen(['ros2','run','bindu_runtime','execution','--ros-args',
        '-r','__ns:='+ns,'-p','run_id:='+run],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    seen={'state':None,'accepted':set()}
    state_sub=node.create_subscription(ExecutionState,ns+'/execution/state',lambda m:seen.update(state=m),10)
    accepted_sub=node.create_subscription(MotionCommand,ns+'/execution/accepted',lambda m:seen['accepted'].add(m.command_id),512)
    publisher=node.create_publisher(MotionCommand,ns+'/execution/targets',8)
    try:
        lease_client=node.create_client(Lease,ns+'/execution/lease')
        submit_client=node.create_client(SubmitMotion,ns+'/execution/submit')
        assert lease_client.wait_for_service(timeout_sec=10)
        assert submit_client.wait_for_service(timeout_sec=5)
        until(node,lambda:seen['state'] and publisher.get_subscription_count()>0)
        lease=wait(node,lease_client.call_async(Lease.Request(operation='acquire',owner=run,resources=['arm'])))
        assert lease.ok
        busy=wait(node,lease_client.call_async(Lease.Request(operation='acquire',owner='conflict',resources=['arm'])))
        assert not busy.ok and busy.code=='BUSY'
        def target(index):
            return MotionCommand(schema_version=1,command_id=run+'_'+str(index),task_id=run,
                profile_hash=lease.profile_hash,lease_id=lease.lease_id,epoch=lease.epoch,
                mode='joint_target',resource_group='arm',stamp=node.get_clock().now().to_msg(),valid_for=.15,
                joint_names=['arm_joint_1','arm_joint_2'],positions=[.2*index/99,.2*index/99])
        wrong=target(0); wrong.joint_names=['wrong','layout']
        reject=wait(node,submit_client.call_async(SubmitMotion.Request(command=wrong)))
        assert not reject.accepted and reject.code=='JOINT_LAYOUT_MISMATCH'
        start=time.monotonic()
        for index in range(100):
            while time.monotonic()<start+index*.01:
                rclpy.spin_once(node,timeout_sec=min(.002,max(0.,start+index*.01-time.monotonic())))
            publisher.publish(target(index))
            rclpy.spin_once(node,timeout_sec=0.)
        until(node,lambda:seen['state'].command_id==run+'_99' and seen['state'].code=='COMMAND_TIMEOUT' and seen['state'].state=='FAILED',3)
        assert len(seen['accepted'])==100,len(seen['accepted'])
        assert not seen['state'].reference.name, 'stopped position reference must be inactive'
        release=wait(node,lease_client.call_async(Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch)))
        assert release.ok
        stale=target(100)
        reject=wait(node,submit_client.call_async(SubmitMotion.Request(command=stale)))
        assert not reject.accepted and reject.code=='INVALID_LEASE'
        return {'case':'online_targets','passed':True,'simulated':True,'accepted_targets':100,
                'checks':['resource conflict','joint layout','100 Hz input','command expiry','old lease rejection']}
    finally:
        terminate(process)
        node.destroy_node()
        log.close()


def timed_chunks(root, output):
    """ROS wire derivatives, revisions, expired prefix and controlled stopping."""
    ns='/bindu_timed_'+uuid.uuid4().hex[:8]
    node=rclpy.create_node('timed_verifier_'+uuid.uuid4().hex[:8])
    logdir=output/'processes'/ns[1:]; logdir.mkdir(parents=True)
    log=(logdir/'execution.log').open('w')
    process=subprocess.Popen(['ros2','run','bindu_runtime','execution','--ros-args',
        '-r','__ns:='+ns],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    seen={'state':None,'samples':[]}
    def receive(m):
        seen['state']=m
        seen['samples'].append(m)
    sub=node.create_subscription(ExecutionState,ns+'/execution/state',receive,100)
    lease_client=node.create_client(Lease,ns+'/execution/lease')
    submit=node.create_client(SubmitMotion,ns+'/execution/submit')
    control=node.create_client(ControlExecution,ns+'/execution/control')
    try:
        assert lease_client.wait_for_service(timeout_sec=10)
        assert submit.wait_for_service(timeout_sec=5)
        until(node,lambda:seen['state'] is not None)
        revision=seen['state'].revision
        lease=wait(node,lease_client.call_async(Lease.Request(operation='acquire',owner=ns,resources=['arm'])))
        assert lease.ok
        until(node,lambda:seen['state'].revision>revision)
        def chunk(identity):
            source=node.get_clock().now().nanoseconds-100000000
            return MotionCommand(schema_version=1,command_id=identity,task_id=ns,
                profile_hash=lease.profile_hash,lease_id=lease.lease_id,epoch=lease.epoch,
                mode='joint_reference_segment',resource_group='arm',
                stamp=Time(sec=source//1000000000,nanosec=source%1000000000),valid_for=1.5,
                expected_revision=seen['state'].revision,joint_names=['arm_joint_1','arm_joint_2'],
                points=[JointTrajectoryPoint(positions=[0.,0.],velocities=[0.,0.],accelerations=[0.,0.],time_from_start=Duration()),
                        JointTrajectoryPoint(positions=[.1,.1],velocities=[.15,.15],accelerations=[0.,0.],time_from_start=Duration(nanosec=600000000))])
        command=chunk('chunk')
        reply=wait(node,submit.call_async(SubmitMotion.Request(command=command)))
        assert reply.accepted,reply.code
        assert reply.revision>command.expected_revision
        assert reply.switch_stamp.sec+reply.switch_stamp.nanosec/1e9 >= command.stamp.sec+command.stamp.nanosec/1e9
        command.command_id='obsolete_revision'
        reject=wait(node,submit.call_async(SubmitMotion.Request(command=command)))
        assert not reject.accepted and reject.code=='STALE_REVISION',reject.code
        until(node,lambda:seen['state'].command_id=='chunk' and seen['state'].state=='FAILED',3)
        assert seen['state'].code=='BUFFER_EXHAUSTED'
        assert not seen['state'].reference.name
        before=seen['state'].revision
        command=chunk('cancel_chunk')
        command.expected_revision=before
        reply=wait(node,submit.call_async(SubmitMotion.Request(command=command)))
        assert reply.accepted,reply.code
        until(node,lambda:seen['state'].command_id=='cancel_chunk' and any(abs(v)>.02 for v in seen['state'].reference.velocity))
        reply=wait(node,control.call_async(ControlExecution.Request(operation='stop',lease_id=lease.lease_id,epoch=lease.epoch)))
        assert reply.ok
        until(node,lambda:seen['state'].command_id=='cancel_chunk' and seen['state'].state=='CANCELED')
        assert not seen['state'].reference.name
        assert any(m.state=='STOPPING' for m in seen['samples'])
        for m in seen['samples']:
            assert len(m.reference.name)==len(m.reference.velocity)==len(m.reference_accelerations)==len(m.reference_jerks)
            assert all(abs(q)<=1.+1e-8 for q in m.reference.position)
            assert all(abs(v)<=1.5+1e-8 for v in m.reference.velocity)
            assert all(abs(a)<=20.+1e-8 for a in m.reference_accelerations)
            assert all(abs(j)<=400.+1e-8 for j in m.reference_jerks)
        return {'case':'timed_chunks','passed':True,'simulated':True,'samples':len(seen['samples']),
            'checks':['wire derivatives','revision acknowledgement','old revision rejected','expired prefix trimmed',
                      'buffer exhaustion','controlled cancel','whole reference layout and bounds']}
    finally:
        terminate(process);node.destroy_node();log.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='artifacts/ros-validation')
    parser.add_argument('--case', action='append', help='Run only named cases')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='116'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    inputs={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('src','tests') for p in (root/folder).rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts}
    (output/'environment.json').write_text(json.dumps({
        'platform':platform.platform(), 'python':sys.version, 'ros_distro':os.environ.get('ROS_DISTRO'),
        'workspace':str(root), 'ros_domain_id':116, 'localhost_only':True,
        'source_sha256':inputs},indent=2))
    rclpy.init()
    results=[]
    cases=[('planner_wheel',dict()),('chunk_wheel',dict(strategy='chunk')),
           ('planner_alternate',dict(profile='alternate_sim')),
           ('chunk_alternate',dict(strategy='chunk',profile='alternate_sim')),
           ('cancel',dict(cancel=True))]+[(f,dict(fault=f)) for f in ('delay','stale','reject','feedback_loss','recorder_loss','perception_loss','hand:reject','hand:feedback_loss')]
    cases.append(('online_targets',{}))
    cases.append(('timed_chunks',{}))
    cases.append(('launch_entry',dict(launch=True)))
    if args.case:
        cases=[item for item in cases if item[0] in args.case]
        assert cases, 'unknown case'
    try:
        for name,kwargs in cases:
            try:
                result=({'online_targets':online_targets,'timed_chunks':timed_chunks}[name](root,output)
                        if name in ('online_targets','timed_chunks') else scenario(root,output,name,**kwargs))
            except Exception as exc:
                result={'case':name,'passed':False,'error':str(exc),'traceback':traceback.format_exc()}
            results.append(result)
            print(json.dumps(result),flush=True)
            (output/'results.json').write_text(json.dumps({'cases':results,'passed':all(r['passed'] for r in results)},indent=2))
    finally: rclpy.shutdown()
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)

if __name__=='__main__': main()
