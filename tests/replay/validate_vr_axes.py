#!/usr/bin/env python3
"""Audit recorded VR clutch geometry with independent six-axis synthetic motions.

Runs the production hand mapping and optional native QP/kinematic devices offline.
No ROS publishers, Isaac commands, vendor drivers or changes to a live session.
Translation and complete rotation must share fixed robot-base axes, including
wrist-only rotation when the initial human/robot shaft directions differ.
"""
from tests.support.paths import ROOT
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0,str(ROOT/'tools'))
from snowbot_errors import pinocchio_fk,angle
from bindu_contracts.teleoperation import VRFrame
from bindu_hardware.drivers.simulated_joints import SimJointDriver,SynchronizedJointProgress
from bindu_kinematics.bimanual import BimanualQP
from bindu_runtime.robot_setup import RobotSetup
from bindu_teleoperation.bimanual_session import BimanualSession
from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT,robot_pose
from bindu_teleoperation.vr.one_euro import rotation_exp,rotation_log


def records(path):
    with path.open() as stream:
        for line in stream:
            try:yield json.loads(line)['data']
            except ValueError:continue


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--input-seq',type=int)
    p.add_argument('--mapping-only',action='store_true')
    p.add_argument('--strict-robot-base',action='store_true',help='Fail if a fixed robot-base rotation axis is remapped')
    p.add_argument('--case',help='One case, e.g. rotation:x:-1')
    p.add_argument('--hold-seconds',type=float,default=.8)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((args.log/'session.json').read_text())
    robot=RobotSetup.load(manifest['configs']['robot_config']['path'])
    cfg=dict(manifest['configs']['tool_config']['content'],urdf=str(robot.urdf))
    capture=None
    for e in records(args.log/'runtime-events.jsonl'):
        if e['state']!='BIMANUAL_CAPTURED':continue
        value=json.loads(e['code'])
        if value['grasp_mode']!='rigid':continue
        if args.input_seq is None or value['input_seq']==args.input_seq:capture=value;break
    if capture is None:raise ValueError('NO_RIGID_CAPTURE')
    hands={}
    for f in records(args.log/'vr-projected.jsonl'):
        if f['seq']==capture['input_seq'] and f['valid']:
            hands[f['side']]=robot_pose(f['pose'])
        if len(hands)==2:break
    if len(hands)!=2:raise ValueError('CAPTURE_HANDS_MISSING')
    hands=[hands[s] for s in ('left','right')]
    human_mid=(hands[0][:3,3]+hands[1][:3,3])/2
    human_axis=hands[0][:3,3]-hands[1][:3,3];human_axis/=np.linalg.norm(human_axis)
    actual0=dict(capture['feedback']);fk=pinocchio_fk(robot.urdf,cfg)
    mount=np.array(cfg['left_to_tool']).reshape(4,4)
    home=fk(actual0)[0]@mount
    template=BimanualSession(cfg,1,0.)
    point=template.control_point_in_tool
    robot_axis=home[:3,:3]@np.array(template.cfg['rod_axis_in_tool'])
    home_mid=home[:3,3]+home[:3,:3]@point
    pan=np.array(cfg['demo']['pan_center_m']);home_pan=home[:3,3]+home[:3,:3]@pan
    cases=[]
    for kind in ('translation','rotation'):
        for index,name in enumerate('xyz'):
            for sign in (-1,1):cases.append((kind,name,sign,np.eye(3)[index]))
    for sign in (-1,1):cases.append(('fixed_wrist_roll','shaft',sign,human_axis))
    if args.case:cases=[c for c in cases if ':'.join(map(str,c[:3]))==args.case]
    if not cases or not math.isfinite(args.hold_seconds) or not .1<=args.hold_seconds<=5.:
        p.error('valid case and 0.1..5 second hold required')
    results=[];trace=(args.output/'samples.jsonl').open('w')
    for kind,name,sign,axis in cases:
        session=BimanualSession(cfg,1,0.);sequence=0
        def feed(t,weight=0.,held=True):
            nonlocal sequence
            sequence+=1
            shift=axis*sign*.02*weight if kind=='translation' else np.zeros(3)
            rotation=rotation_exp(axis*sign*math.radians(10)*weight) if kind!='translation' else np.eye(3)
            frames=[]
            for side,hand in zip(('left','right'),hands):
                pose=hand.copy()
                pose[:3,3]=(hand[:3,3] if kind=='fixed_wrist_roll' else
                    human_mid+rotation@(hand[:3,3]-human_mid))+shift
                pose[:3,:3]=rotation@hand[:3,:3]
                raw=OPENXR_TO_ROBOT.T@pose@OPENXR_TO_ROBOT
                frames.append(VRFrame('offline_axes',sequence,t,side,tuple(raw.ravel()),
                    grip=float(held),mapping='pico_snowbot_v1'))
            session.ingest(*frames,t)
        for t,held in ((1.,False),(1.02,True),(2.1,True)):feed(t,held=held)
        session.capture(home)
        feed(2.12,1.)
        target=session.target.copy();target_mid=target[:3,3]+target[:3,:3]@point
        target_pan=target[:3,3]+target[:3,:3]@pan
        rv=rotation_log(target[:3,:3]@home[:3,:3].T)
        intended_axis=axis
        expected_rotation=(np.eye(3) if kind=='translation' else rotation_exp(axis*sign*math.radians(10)))
        direction_error=0. if kind=='translation' else math.degrees(math.acos(float(np.clip(
            (rv@intended_axis*sign)/max(np.linalg.norm(rv),1e-15),-1.,1.))))
        result=dict(kind=kind,axis=name,sign=sign,
            target_midpoint_delta_mm=((target_mid-home_mid)*1000).tolist(),
            target_pan_delta_mm=((target_pan-home_pan)*1000).tolist(),
            target_rotation_vector_deg=np.degrees(rv).tolist(),
            robot_base_rotation_axis_error_deg=direction_error,
            mapping_rotation_error_deg=angle(expected_rotation.T@target[:3,:3]@home[:3,:3].T),
            mapping_translation_error_mm=float(np.linalg.norm(target_mid-home_mid-
                (axis*sign*.02 if kind=='translation' else 0)))*1000,
            kinematics=None)
        if not args.mapping_only:
            qp=BimanualQP(cfg,robot.profile);actual=dict(actual0)
            drivers=[SimJointDriver(robot.profile.groups[g],1.,{n:actual[n] for n in robot.profile.groups[g]}) for g in cfg['resource_groups']]
            SynchronizedJointProgress(drivers,{n:robot.profile.speed(n) for d in drivers for n in d.positions})
            reference=[actual[n] for n in qp.names];failure=None;limited=0;max_ms=0.
            qp.capture(actual)
            # Recapture resets the synthetic input and target to the same home.
            session=BimanualSession(cfg,1,0.);sequence=0
            for t,held in ((1.,False),(1.02,True),(2.1,True)):feed(t,held=held)
            session.capture(home)
            try:
                weights=np.r_[np.linspace(0.,1.,61)[1:],np.ones(round(args.hold_seconds/.02))]
                for i,weight in enumerate(weights):
                    t=2.12+i*.02;feed(t,float(weight))
                    at=time.perf_counter()
                    reference,_=qp.solve(actual,session.target,reference=reference,dt=.02,target_stamp=t)
                    max_ms=max(max_ms,(time.perf_counter()-at)*1000)
                    limited+=bool(qp.last_geometry_step)
                    for d in drivers:d.write_positions({n:v for n,v in zip(qp.names,reference) if n in d.positions})
                    for d in drivers:actual.update(d.read(t,.02).positions)
                    pose=fk(actual)[0]@mount
                    trace.write(json.dumps(dict(case=f'{kind}:{name}:{sign}',step=i,target=session.target.tolist(),
                        feedback_tool=pose.tolist(),feedback_grip_midpoint=(pose[:3,3]+pose[:3,:3]@point).tolist()))+'\n')
            except (ValueError,RuntimeError) as exc:failure=str(exc)
            finally:qp.close()
            pose=fk(actual)[0]@mount;mid=pose[:3,3]+pose[:3,:3]@point
            result['kinematics']=dict(failure=failure,limited=limited,max_solve_ms=max_ms,
                joint_positions=actual,
                midpoint_delta_mm=((mid-home_mid)*1000).tolist(),
                rotation_vector_deg=np.degrees(rotation_log(pose[:3,:3]@home[:3,:3].T)).tolist(),
                midpoint_tracking_error_mm=float(np.linalg.norm(mid-target_mid))*1000,
                rotation_tracking_error_deg=angle(target[:3,:3].T@pose[:3,:3]))
        results.append(result)
        print(kind,name,sign,'axis error',round(direction_error,3),'midpoint mm',np.round(result['target_midpoint_delta_mm'],3),
            'QP',result['kinematics'],flush=True)
    trace.close()
    result=dict(source_run=manifest['run_id'],input_seq=capture['input_seq'],
        control_dt_s=.02,hold_seconds=args.hold_seconds,
        scope='offline production mapping + native QP + synchronized kinematic devices; no live outputs',
        human_axis_in_robot=human_axis.tolist(),robot_shaft_axis=robot_axis.tolist(),
        initial_axis_difference_deg=math.degrees(math.acos(float(np.clip(human_axis@robot_axis,-1.,1.)))),
        grip_midpoint_in_tool=point.tolist(),cases=results,
        checks=dict(translation_correct=all(c['mapping_translation_error_mm']<1e-6 for c in results),
            fixed_robot_rotation_axes=all(c['robot_base_rotation_axis_error_deg']<.1 and c['mapping_rotation_error_deg']<1e-4 for c in results),
            kinematic_faults=[c for c in results if c['kinematics'] and c['kinematics']['failure']]))
    (args.output/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if result['checks']['kinematic_faults'] or not result['checks']['translation_correct']:
        raise SystemExit(1)
    if args.strict_robot_base and not result['checks']['fixed_robot_rotation_axes']:
        raise SystemExit(1)


if __name__=='__main__':main()
