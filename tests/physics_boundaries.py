"""IK boundary and independent PhysX link-pose checks for the G1 fixture."""
import json
import math
from pathlib import Path
from collections import OrderedDict
import numpy as np
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_interfaces.msg import SimulationFeedback
from rclpy.qos import QoSProfile,ReliabilityPolicy
from bindu_contracts.teleoperation import IKRequest
from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
from bindu_teleoperation.config import load_config
from validate_ros import until


def matrix(pose):
    x,y,z,w=pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w
    q=np.array([x,y,z,w]);q/=np.linalg.norm(q);x,y,z,w=q
    out=np.eye(4);out[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
    out[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    return out


def boundary_checks(node,args,profile,seen,results,command,send,finished,release,drain):
    root=Path(__file__).resolve().parents[1]
    solvers={side:PinocchioCasadiIK(load_config(root/f'src/integration/bindu_runtime/config/teleop_g1_{side}.json',
        profile,root/'src/hardware/bindu_description/urdf')['kinematics']) for side in ('left','right')}
    pending_q=OrderedDict();pending_pose=OrderedDict();comparisons=[]
    def key(m):return (m.sec,m.nanosec)
    def compare(k):
        if k not in pending_q or k not in pending_pose:return
        qmsg,poses=pending_q.pop(k),pending_pose.pop(k)
        assert poses.header.frame_id=='world' and len(poses.poses)==3
        measured=dict(zip(qmsg.joints.name,qmsg.joints.position));base_inv=np.linalg.inv(matrix(poses.poses[0]))
        for index,(side,solver) in enumerate(solvers.items(),1):
            arm=solver.arm;expected=arm.fk([measured[n] for n in arm.names],[measured[n] for n in arm.context_names])
            actual=base_inv@matrix(poses.poses[index])
            pe=float(np.linalg.norm(expected[:3,3]-actual[:3,3]))
            re=float(np.arccos(np.clip((np.trace(expected[:3,:3].T@actual[:3,:3])-1)/2,-1,1)))
            comparisons.append({'stamp':k[0]+k[1]/1e9,'side':side,'position_error_m':pe,'rotation_error_rad':re,
                'physx_pose':actual.ravel().tolist(),'fk_pose':expected.ravel().tolist()})
    def qreceive(m):
        k=key(m.stamp);pending_q[k]=m;compare(k)
        while len(pending_q)>300:pending_q.popitem(last=False)
    def preceive(m):
        k=key(m.header.stamp);pending_pose[k]=m;compare(k)
        while len(pending_pose)>300:pending_pose.popitem(last=False)
    qos=QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT)
    qs=node.create_subscription(SimulationFeedback,args.namespace+'/simulation/feedback',qreceive,qos)
    ps=node.create_subscription(PoseArray,args.namespace+'/simulation/link_poses',preceive,qos)
    try:
        until(node,lambda:len(comparisons)>=10,seconds=10.)
        for side,solver in solvers.items():
            arm=solver.arm
            def request(target):
                values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
                now=node.get_clock().now().nanoseconds/1e9
                return IKRequest(side,1,now,now+.2,arm.names,tuple(values[n] for n in arm.names),
                    tuple(target.ravel()),tuple(values[n] for n in arm.context_names))
            values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
            seed=np.array([values[n] for n in arm.names]);context=[values[n] for n in arm.context_names]
            full=arm.pin.neutral(arm.full);full[arm.full_indices]=seed;full[arm.context_indices]=context
            jac=arm.pin.computeFrameJacobian(arm.full,arm.full_data,full,arm.full.getFrameId(arm.cfg['ee_link']),arm.pin.LOCAL_WORLD_ALIGNED)[:,arm.full_indices]
            singular=float(np.linalg.svd(jac,compute_uv=False)[-1]);assert singular<.02,singular
            guide=seed.copy();guide[0]+=.04;guide[3]+=-.03 if side=='left' else .03
            target=arm.fk(guide,context);solved=solver.solve(request(target));assert solved.success,solved.code
            cmd=command(side+'_arm','finite_trajectory');cmd.joint_names=list(arm.names)
            cmd.points=[JointTrajectoryPoint(positions=list(solved.positions),time_from_start=Duration(sec=2))]
            send(cmd);finished(cmd);release();drain(.3)
            results.append({'case':side+'_near_singular_small_step','passed':True,'jacobian_min_singular_value':singular,
                'ik_position_error_m':solved.position_error,'max_solution_step_rad':float(np.max(np.abs(np.array(solved.positions)-seed)))})
            # A large reachable pose may still violate the continuity contract.
            guide[0]+=.9;guide[3]+=-.4 if side=='left' else .4
            jump=solver.solve(request(arm.fk(guide,context)))
            assert not jump.success and jump.code in ('IK_DISCONTINUITY','IK_RESIDUAL','IK_SOLVER_FAILED','IK_TIMEOUT'),jump
            results.append({'case':side+'_abrupt_target_rejected','passed':True,'code':jump.code})
            # Sweep outwards from the near-straight arm, retaining rejected cases.
            current=arm.fk(request(target).seed,request(target).context)
            direction=current[:3,3]-arm.full_data.oMf[arm.full.getFrameId(side+'_arm_base_link')].translation
            direction/=np.linalg.norm(direction)
            sweep=[]
            for offset in (.0,.01,.03,.08):
                goal=current.copy();goal[:3,3]+=offset*direction
                solved=solver.solve(request(goal))
                sweep.append({'outward_m':offset,'success':solved.success,'code':solved.code,
                    'position_error_m':solved.position_error if solved.success else None})
            assert any(not r['success'] for r in sweep),sweep
            results.append({'case':side+'_workspace_edge_sweep','passed':True,'samples':sweep})
            # Broader visible articulation movement exercises the independent
            # position/orientation comparison well away from the initial pose.
            target=[values[n] for n in arm.names];target[0]=.45 if side=='left' else -.45;target[3]=-.3 if side=='left' else .3
            cmd=command(side+'_arm','finite_trajectory');cmd.joint_names=list(arm.names)
            cmd.points=[JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=3))]
            send(cmd);finished(cmd);drain(.3);release();drain(.3)
        assert len(comparisons)>=100,len(comparisons)
        maxp=max(x['position_error_m'] for x in comparisons);maxr=max(x['rotation_error_rad'] for x in comparisons)
        assert maxp<.002 and maxr<.005,(maxp,maxr)
        results.append({'case':'independent_physx_link_pose_vs_fk','passed':True,'samples':len(comparisons),
            'max_position_error_m':maxp,'max_rotation_error_rad':maxr,'frame':'base_link','source':'PhysX tensor link transforms'})
    finally:
        node.destroy_subscription(qs);node.destroy_subscription(ps)
        (args.output/'link-pose-comparisons.json').write_text(json.dumps(comparisons)+'\n')
