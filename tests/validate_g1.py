#!/usr/bin/env python3
"""Launch the provisional whole-body model; exercise measured joints and TF."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from bindu_contracts.profile import Profile
from bindu_interfaces.msg import ExecutionState, MotionCommand, RecorderHealth
from bindu_interfaces.srv import Lease, SubmitMotion, InjectFault, ControlExecution
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_runtime.common import stamp, seconds
from validate_ros import wait, until, terminate


def run(output):
    root = Path(__file__).resolve().parents[1]
    profile = Profile.load(root/'src/integration/bindu_runtime/config/g1_provisional_sim.json')
    ns = '/g1_'+uuid.uuid4().hex[:8]
    node = rclpy.create_node('g1_validator')
    seen = {'state': None, 'joints': None, 'health': None, 'tf': {}, 'static': {}}
    joint_samples = {}
    def sample_key(stamp):
        return (stamp.sec, stamp.nanosec)
    def joints(msg):
        seen['joints'] = msg
        joint_samples[sample_key(msg.header.stamp)] = dict(zip(msg.name, msg.position))
        while len(joint_samples)>300:
            del joint_samples[next(iter(joint_samples))]
    def transforms(msg, key):
        seen[key].update({t.child_frame_id:t for t in msg.transforms})
    node.create_subscription(ExecutionState, ns+'/execution/state', lambda m:seen.update(state=m), 10)
    node.create_subscription(JointState, ns+'/joint_states', joints, 10)
    node.create_subscription(RecorderHealth, ns+'/recorder/health', lambda m:seen.update(health=m), 5)
    node.create_subscription(TFMessage, ns+'/tf', lambda m:transforms(m,'tf'), 100)
    node.create_subscription(TFMessage, ns+'/tf_static', lambda m:transforms(m,'static'),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients = {name:node.create_client(kind, ns+'/execution/'+name)
               for name,kind in [('lease',Lease),('submit',SubmitMotion),('sim_fault',InjectFault),('control',ControlExecution)]}
    results = []
    log = (output/'launch.log').open('w')
    process = subprocess.Popen(['ros2','launch','bindu_runtime','g1_sim.launch.py','namespace:='+ns,
                                'output:='+str(output/'episodes')], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        for client in clients.values(): assert client.wait_for_service(timeout_sec=15)
        until(node, lambda:seen['joints'] and seen['health'] and seen['health'].ready and
              'base_link' in seen['tf'] and 'leg_link5' in seen['static'], seconds=15)
        assert len(seen['joints'].name) == 19
        results.append({'case':'description_and_19_joint_feedback','passed':True})
        for group,names in profile.groups.items():
            before = dict(zip(seen['state'].joints.name, seen['state'].joints.position))
            lease = wait(node, clients['lease'].call_async(Lease.Request(operation='acquire', owner='g1_test', resources=[group])))
            assert lease.ok, lease.code
            command = MotionCommand(schema_version=1, command_id=uuid.uuid4().hex, lease_id=lease.lease_id,
                epoch=lease.epoch, profile_hash=profile.digest, task_id='g1_test', mode='finite_trajectory',
                resource_group=group, stamp=stamp(node.get_clock().now().nanoseconds/1e9), valid_for=1.8,
                joint_names=names, points=[JointTrajectoryPoint(positions=[.04]*len(names), time_from_start=Duration(sec=1))])
            reply = wait(node, clients['submit'].call_async(SubmitMotion.Request(command=command)))
            assert reply.accepted, reply.code
            until(node, lambda:seen['state'].command_id==command.command_id and seen['state'].state=='SUCCEEDED', seconds=1.7)
            positions = dict(zip(seen['state'].joints.name, seen['state'].joints.position))
            assert all(abs(v-(.04 if n in names else before[n]))<1e-5 for n,v in positions.items())
            assert wait(node, clients['lease'].call_async(Lease.Request(operation='release', lease_id=lease.lease_id, epoch=lease.epoch))).ok
            results.append({'case':group+'_execution_isolation','passed':True})
        frames = ('left_arm_link7','right_arm_link7','head_link2','leg_link3')
        barrier = seconds(seen['state'].feedback_stamp)
        until(node, lambda:all(n in seen['tf'] and seconds(seen['tf'][n].header.stamp)>=barrier and
                              sample_key(seen['tf'][n].header.stamp) in joint_samples for n in frames))
        # TF rotation for each scalar joint must correspond to its measured angle.
        import numpy as np
        import pinocchio as pin
        model = pin.buildModelFromUrdf(str(root/'src/hardware/bindu_description/urdf/g1_provisional.urdf'))
        data = model.createData(); q = pin.neutral(model)
        for child in frames:
            t = seen['tf'][child]
            # Compare the exact measured sample carried by this TF, not a target
            # or a newer feedback sample from an independently scheduled topic.
            for name,value in joint_samples[sample_key(t.header.stamp)].items():
                q[model.joints[model.getJointId(name)].idx_q] = value
            pin.framesForwardKinematics(model, data, q)
            expected = data.oMf[model.getFrameId(t.header.frame_id)].inverse()*data.oMf[model.getFrameId(child)]
            v = t.transform.translation; rot = t.transform.rotation
            np.testing.assert_allclose([v.x,v.y,v.z], expected.translation, atol=1e-7)
            np.testing.assert_allclose(pin.Quaternion(rot.w,rot.x,rot.y,rot.z).matrix(), expected.rotation, atol=1e-7)
        results.append({'case':'measured_joint_tf_matches_fk','passed':True})
        lease = wait(node, clients['lease'].call_async(Lease.Request(operation='acquire', owner='base_test', resources=['base'])))
        assert lease.ok
        cmd = MotionCommand(schema_version=1,command_id=uuid.uuid4().hex,lease_id=lease.lease_id,epoch=lease.epoch,
            profile_hash=profile.digest,task_id='base_test',mode='base_velocity',resource_group='base',
            stamp=stamp(node.get_clock().now().nanoseconds/1e9),valid_for=1.2,duration=.6)
        cmd.velocity.linear.x=.1;cmd.velocity.angular.z=.2
        assert wait(node,clients['submit'].call_async(SubmitMotion.Request(command=cmd))).accepted
        until(node, lambda:seen['state'].base_y>.0001 and seen['tf']['base_link'].transform.translation.y>.0001)
        reply = wait(node,clients['control'].call_async(ControlExecution.Request(operation='stop',lease_id=lease.lease_id,epoch=lease.epoch)))
        assert reply.ok
        until(node,lambda:seen['state'].base_velocity.linear.x==0. and seen['state'].base_velocity.angular.z==0.)
        assert wait(node, clients['lease'].call_async(Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch))).ok
        results.append({'case':'differential_odom_and_stop','passed':True})
        # Frozen base feedback must not acquire new timestamps in joint_states or TF.
        assert wait(node,clients['sim_fault'].call_async(InjectFault.Request(fault='base:feedback_loss'))).ok
        end=time.monotonic()+.35
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        stamps=(seconds(seen['joints'].header.stamp),seconds(seen['tf']['base_link'].header.stamp))
        end=time.monotonic()+.35
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert stamps==(seconds(seen['joints'].header.stamp),seconds(seen['tf']['base_link'].header.stamp))
        results.append({'case':'frozen_feedback_not_restamped','passed':True})
    finally:
        terminate(process);log.close();node.destroy_node()
        (output/'results.json').write_text(json.dumps(results,indent=2))
    return results


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try:print(json.dumps(run(args.output)),flush=True)
    finally:rclpy.shutdown()
