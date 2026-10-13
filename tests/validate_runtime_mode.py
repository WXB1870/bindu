#!/usr/bin/env python3
"""Verify mode selection using local ROS only; real endpoint stays read-only."""
import argparse
import gzip
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_prefix
from bindu_interfaces.msg import ExecutionState, MotionCommand, RecorderHealth
from bindu_interfaces.srv import Lease, SubmitMotion
from bindu_contracts.profile import Profile
from bindu_runtime.common import stamp
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def until(node, predicate, timeout=8.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate(): return
    raise AssertionError('MODE_ROS_TIMEOUT')


def call(node, client, request):
    until(node, client.service_is_ready)
    future = client.call_async(request); until(node, future.done)
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('ISOLATED_LOCALHOST_DOMAIN_REQUIRED')
    output = args.output.resolve();output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    rclpy.init();node = rclpy.create_node('mode_verifier_'+uuid.uuid4().hex[:8])
    processes, handles, outcomes = [], [], []
    token = uuid.uuid4().hex[:8]
    states, health = {}, {}
    contexts = {}
    for mode in ('simulation', 'real'):
        ns = '/mode_'+mode+'_'+token
        states[mode], health[mode] = [], []
        node.create_subscription(ExecutionState, ns+'/execution/state', lambda m,k=mode: states[k].append(m), 10)
        node.create_subscription(RecorderHealth, ns+'/recorder/health', lambda m,k=mode: health[k].append(m), 5)
        recipe = {'schema_version':2,'robot_launch':'snowbot_adapter.launch.py','arguments':{
            'runtime_mode':mode,'namespace':ns,'run_id':mode+'_'+token,
            'vendor_namespace':'/mode_vendor_'+mode+'_'+token,'command_enabled':False,
            'recording_mode':'normal','output':str(output/'recordings')}}
        path=output/(mode+'.json');path.write_text(json.dumps(recipe))
        log=(output/(mode+'.log')).open('w');handles.append(log)
        p=subprocess.Popen(['ros2','launch','bindu_runtime','runtime.launch.py','config:='+str(path)],
                           stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p)
        contexts[mode] = (ns, recipe, node.create_client(Lease, ns+'/execution/lease'),
                         node.create_client(SubmitMotion, ns+'/execution/submit'))
    try:
        until(node, lambda: all(states[k] and health[k] for k in states))
        for mode in states:
            s = states[mode][-1]
            assert s.runtime_mode == mode and s.simulated == (mode == 'simulation')
            assert s.device_backend == ('kinematic' if mode == 'simulation' else 'snowbot_vendor')
            assert health[mode][-1].runtime_mode == mode
            vendor=contexts[mode][1]['arguments']['vendor_namespace']
            assert all(node.count_publishers(vendor+'/'+name)==0 for name in
                       ('joint_position_cmd','body/motor_cmd','head/motor_cmd','bindu/heartbeat'))
            manifest=output/'recordings'/contexts[mode][1]['arguments']['run_id']/'manifest.json'
            data=json.loads(manifest.read_text())
            assert data['runtime_mode']==mode and data['simulated']==(mode=='simulation')
        outcomes.append('one_recipe_switch_selects_backend_state_events_and_recording')
        outcomes.append('simulation_and_readonly_real_have_no_vendor_command_writers')
        for mode in states:
            ns, recipe, lease_client, submit_client=contexts[mode]
            cmd=MotionCommand(runtime_mode='real' if mode=='simulation' else 'simulation')
            reply=call(node,submit_client,SubmitMotion.Request(command=cmd))
            assert not reply.accepted and reply.code=='RUNTIME_MODE_MISMATCH',reply.code
        outcomes.append('opposite_mode_commands_rejected_before_lease_or_dispatch')
        ns, recipe, lease_client, submit_client=contexts['real']
        reply=call(node,submit_client,SubmitMotion.Request(command=MotionCommand()))
        assert not reply.accepted and reply.code=='RUNTIME_MODE_REQUIRED_FOR_REAL_COMMAND'
        reply=call(node,lease_client,Lease.Request(operation='acquire',owner='probe',resources=['left_arm','right_arm']))
        assert not reply.ok and 'COMMANDS_DISABLED' in reply.code
        outcomes.append('real_requires_explicit_command_mode_and_separate_write_permission')
        ns, recipe, lease_client, submit_client=contexts['simulation']
        lease=call(node,lease_client,Lease.Request(operation='acquire',owner='probe',resources=['left_arm']))
        assert lease.ok,lease.code
        profile=Profile.load(root/'src/integration/bindu_runtime/config/snowbot_shovel_sim.json')
        names=profile.groups['left_arm'];q=dict(zip(states['simulation'][-1].joints.name,states['simulation'][-1].joints.position))
        targets=[q[n] for n in names];targets[0]+=.01
        cmd=MotionCommand(schema_version=1,command_id='legacy_'+token,task_id='mode_probe',lease_id=lease.lease_id,
            epoch=lease.epoch,profile_hash=profile.digest,mode='joint_target',resource_group='left_arm',
            stamp=stamp(node.get_clock().now().nanoseconds*1e-9),valid_for=.3,joint_names=names,positions=targets)
        reply=call(node,submit_client,SubmitMotion.Request(command=cmd));assert reply.accepted,reply.code
        outcomes.append('legacy_untagged_simulation_commands_remain_compatible')
        set_client=node.create_client(SetParameters,ns+'/execution/set_parameters')
        reply=call(node,set_client,SetParameters.Request(parameters=[Parameter(name='runtime_mode',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value='real'))]))
        assert not reply.results[0].successful
        get_client=node.create_client(GetParameters,ns+'/execution/get_parameters')
        values=call(node,get_client,GetParameters.Request(names=['runtime_mode','simulation'])).values
        assert values[0].string_value=='simulation' and values[1].bool_value
        outcomes.append('runtime_mode_cannot_change_while_execution_is_active')
        ns=contexts['real'][0]
        foreign=node.create_publisher(ExecutionState,ns+'/execution/state',1)
        foreign.publish(ExecutionState(runtime_mode='simulation',simulated=True,code='FOREIGN_SIMULATION'))
        until(node,lambda: health['real'] and health['real'][-1].error=='RECORDING_RUNTIME_MODE_MISMATCH')
        assert not health['real'][-1].ready
        outcomes.append('real_recorder_rejects_simulation_frames_and_reports_contamination')
        # Constructor conflicts exit before creating any actuator transport.
        executable=Path(get_package_prefix('bindu_runtime'))/'lib/bindu_runtime/snowbot_adapter'
        for label, parameters, reason in [
            ('conflict',['runtime_mode:=real','simulation:=true'],'RUNTIME_MODE_CONFLICT'),
            ('vendor_simulation',['runtime_mode:=simulation'],'SNOWBOT_VENDOR_REQUIRES_REAL_MODE')]:
            log=output/(label+'.log')
            with log.open('w') as f:
                command=[str(executable),'--ros-args','-r','__ns:=/invalid_'+token,
                         '-p','profile:='+str(root/'src/integration/bindu_runtime/config/snowbot_vendor_position.json')]
                for parameter in parameters:command+=['-p',parameter]
                result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=6.)
            assert result.returncode!=0 and reason in log.read_text()
        outcomes.append('conflicting_alias_and_vendor_simulation_exit_before_device_assembly')
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        deadline=time.monotonic()+8.
        while any(p.poll() is None for p in processes) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)
        for handle in handles:handle.close()
        node.destroy_node();rclpy.shutdown()
    real_log=output/'recordings'/contexts['real'][1]['arguments']['run_id']/'episode.jsonl.gz'
    with gzip.open(real_log,'rt') as stream:
        records=[json.loads(line) for line in stream]
    assert records and any(r['kind']=='feedback' for r in records)
    assert all(r['data'].get('runtime_mode')=='real' for r in records)
    result={'scope':'localhost synthetic only; no vendor driver/hardware; real entry read-only',
            'checks':outcomes,'real_record_count':len(records),'hardware_started':False}
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
