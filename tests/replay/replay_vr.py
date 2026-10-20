#!/usr/bin/env python3
"""Replay recorded hand poses through mapping/QP and synchronized kinematic IO.

No ROS publishers, vendor drivers, or live robot outputs. Each captured segment
starts from its recorded measured joints. Received time preserves transport
gaps; client time removes only transport jitter, without smoothing hand poses.
Workflow/lease/stop acknowledgement need the separate ROS integration gate.
"""
from tests.support.paths import ROOT
import argparse
from collections import Counter
from dataclasses import replace
import gc
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from ament_index_python.packages import get_package_prefix
from bindu_contracts.teleoperation import VRFrame
from bindu_hardware.drivers.simulated_joints import SimJointDriver, SynchronizedJointProgress
from bindu_kinematics.bimanual import BimanualQP
from bindu_kinematics.tool_geometry import make_collision_checker, coupling_metrics
from bindu_runtime.robot_setup import RobotSetup
from bindu_teleoperation.bimanual_session import BimanualSession
from bindu_teleoperation.independent_session import IndependentArmsSession

def records(path):
    chunks=[path]+sorted(path.parent.glob(path.stem+'-[0-9][0-9][0-9][0-9]'+path.suffix))
    for chunk in chunks:
        with chunk.open() as stream:
            for line in stream:
                try:yield json.loads(line)['data']
                except ValueError:continue  # A writer may have an incomplete last line.


def seconds(stamp):
    return stamp['sec']+stamp['nanosec']*1e-9


def frame(d, stamp):
    return VRFrame(d['source_id'],d['seq'],stamp,d['side'],tuple(d['pose']),
        d['grip'],0.,valid=d['valid'],stop=d.get('stop',False),mapping='pico_snowbot_v1',
        clutch_seq=d['clutch_seq'],virtual_grip=d.get('virtual_grip',False))


def dataset(folder,until):
    pairs={}
    for d in records(folder/'vr-raw.jsonl'):
        if seconds(d['stamp'])<=until:pairs.setdefault((d['source_id'],d['seq']),{})[d['side']]=d
    pairs={k:(v['left'],v['right']) for k,v in pairs.items() if set(v)=={'left','right'}}
    clocks={}
    for d in records(folder/'vr-health.jsonl'):
        if d.get('stamp',float('inf'))<=until and d.get('stage')=='controller_timing' and d.get('client_timing_available'):
            clocks[(d['source_id'],d['seq'])]=d['client']['perf_ms']*.001
    segments=[];current=None
    for e in records(folder/'runtime-events.jsonl'):
        if seconds(e['stamp'])>until:continue
        try:c=json.loads(e['code'])
        except ValueError:continue
        if e['state']=='BIMANUAL_CAPTURED':
            if current:current['end']=seconds(e['stamp'])
            current=dict(capture=c,start=seconds(e['stamp']),end=float('inf'))
            segments.append(current)
        elif e['state']=='BIMANUAL_ENDED' and current:
            current['end']=seconds(e['stamp']);current['recorded_end']=c['code'];current=None
    for segment in segments:
        c=segment['capture'];seq=c['input_seq']
        candidates=[k for k in pairs if k[1]==seq]
        if not candidates:raise ValueError('CAPTURE_INPUT_MISSING')
        key=min(candidates,key=lambda k:abs(seconds(pairs[k][0]['stamp'])-c['input_stamp']))
        if abs(seconds(pairs[key][0]['stamp'])-c['input_stamp'])>.05:
            raise ValueError('CAPTURE_SOURCE_AMBIGUOUS')
        segment['pairs']=[pairs[k] for k in sorted(pairs,key=lambda k:seconds(pairs[k][0]['stamp']))
            if k[0]==key[0] and k[1]>=seq and seconds(pairs[k][0]['stamp'])<=segment['end']]
    return segments,clocks


def replay(segment,clocks,cfg,profile,clock):
    c=segment['capture'];ind=c['grasp_mode'].startswith('independent:')
    qp=BimanualQP(cfg,profile)
    result=dict(input_seq=c['input_seq'],mode=c['grasp_mode'],recorded_end=segment.get('recorded_end'),
                frames=0,limited=0,held=0,max_solve_ms=0.,failure=None,limits=Counter())
    errors=[];free_errors=[];samples=[]
    try:
        if not segment['pairs']:raise ValueError('NO_CAPTURE_INPUT')
        if ind:
            recipe=dict(cfg['demo']);recipe['left_grip_in_tool_m']=(np.linalg.inv(qp.tool) @ np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
            qp.empty_collision_check=make_collision_checker(Path(cfg['urdf']).parent/recipe['display_urdf'],
                recipe,conservative_convex=True,tool_present=False,validation_only=True)
        actual=dict(c['feedback']);reference=[actual[n] for n in qp.names]
        capture_joints={n:actual[n] for n in qp.names}
        previous_joints=dict(capture_joints)
        motion=result['joint_motion']=dict(
            units='model joint position units',
            travel={n:0. for n in qp.names},max_from_capture={n:0. for n in qp.names})
        session=(IndependentArmsSession if ind else BimanualSession)(cfg,1,-10.)
        first=segment['pairs'][0];t0=seconds(first[0]['stamp'])
        client0=clocks.get((first[0]['source_id'],first[0]['seq']))
        if clock=='client' and client0 is None:raise ValueError('CLIENT_CLOCK_MISSING')
        # Restore a stationary released/pressed baseline before the logged
        # capture. These three setup frames do not count as replayed motion.
        initial=[replace(frame(d,1.),virtual_grip=c['grasp_mode']=='right_axial') for d in first]
        for at,seq,released in ((1.,0,True),(1.02,1,False),(2.1,2,False)):
            pair=[replace(f,stamp=at,seq=seq,grip=0. if released else f.grip) for f in initial]
            session.ingest(*pair,at)
        home=qp.capture_independent(actual) if ind else qp.capture(actual,c['grasp_mode'])
        session.capture(home)
        # The field capture stores the mapping anchor after asynchronous FK.
        # Preserve it instead of silently choosing a different hand origin.
        anchor=np.asarray(c['controller_pose']).reshape((2,4,4) if ind else (4,4))
        session.anchor_input=anchor.copy()
        saved_filter=c.get('pose_filter_state') if c.get('pose_filter')==cfg.get('pose_filter') else None
        if ind:
            session.anchors=[anchor[i].copy() if session.active_mask&(1<<i) else None for i in range(2)]
            if c.get('controller_anchors') is not None:
                session.anchors=[np.asarray(pose) if pose is not None else None for pose in c['controller_anchors']]
                session.robot_anchors=[np.asarray(pose) if pose is not None else None for pose in c['robot_anchors']]
            for i,hand in enumerate(session.hands):
                if hand.pose_filter:
                    hand.pose_filter.reset()
                    hand.filtered_pose=hand.pose_filter.update(anchor[i],session.stamp)
                    if saved_filter is not None and saved_filter[i] is not None:
                        hand.pose_filter.restore(saved_filter[i],stamp=session.stamp)
                        hand.filtered_pose=hand.pose_filter.filtered.copy()
                    session.current_input[i]=hand.filtered_pose
        else:
            session.rod_anchor_basis=anchor[:3,:3].copy()
            if session.pose_filter:
                session.pose_filter.reset()
                session.current_input=session.pose_filter.update(anchor,session.stamp)
                if saved_filter is not None:
                    session.pose_filter.restore(saved_filter,stamp=session.stamp)
                    session.current_input=session.pose_filter.filtered.copy()
        session.seq=-1;session.input_stamps=None
        drivers=[SimJointDriver(profile.groups[g],1.,{n:actual[n] for n in profile.groups[g]}) for g in cfg['resource_groups']]
        SynchronizedJointProgress(drivers,{n:profile.speed(n) for d in drivers for n in d.positions})
        import pinocchio as pin
        model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData()
        ids={cfg[k+'_frame']:model.getFrameId(cfg[k+'_frame']) for k in ('left','right')}
        def sample(t):
            q=pin.neutral(model)
            for n,v in actual.items():
                if model.existJointName(n):q[model.joints[model.getJointId(n)].idx_q]=v
            pin.framesForwardKinematics(model,data,q)
            return dict(t=t,flanges={n:data.oMf[i].homogeneous.copy() for n,i in ids.items()})
        samples=[sample(0.)];previous=2.1;last_mask=session.grasp_mode
        gc.collect()
        for index,pair in enumerate(segment['pairs']):
            key=(pair[0]['source_id'],pair[0]['seq'])
            if clock=='client' and key not in clocks:raise ValueError('CLIENT_CLOCK_MISSING')
            now=2.1+(clocks[key]-client0 if clock=='client' else seconds(pair[0]['stamp'])-t0)
            if now<=previous:continue
            gap=now-previous
            if gap>cfg['input_max_age']:raise ValueError('RECORDED_INPUT_TIMEOUT')
            frames=[replace(frame(d,now),seq=index+10,virtual_grip=c['grasp_mode']=='right_axial') for d in pair]
            if any(f.stop for f in frames):
                result['ended_by']='RECORDED_OPERATOR_STOP';break
            session.ingest(*frames,now)
            if session.mode!='follow' or session.grasp_mode!=last_mask:
                result['ended_by']='CLUTCH_OR_GRASP_TRANSITION';break
            before=time.perf_counter()
            if ind:
                positions,_=qp.solve_independent(actual,session.target,active_mask=session.active_mask,
                    reference=reference,dt=min(gap,.1))
            else:
                positions,_=qp.solve(actual,session.target,reference=reference,dt=min(gap,.1),target_stamp=now)
            result['max_solve_ms']=max(result['max_solve_ms'],(time.perf_counter()-before)*1000)
            diagnostic=qp.last_geometry_step
            if diagnostic:
                result['limited']+=1;result['held']+=diagnostic['scale']==0
                result['limits'][diagnostic['first_rejection'].split(':')[0]]+=1
            reference=positions
            for driver in drivers:driver.write_positions({n:v for n,v in zip(qp.names,positions) if n in driver.positions})
            # Use the same synchronized progress implementation as kinematic IO.
            remaining=gap;tick=previous
            while remaining>1e-9:
                dt=min(.02,remaining);tick+=dt
                for driver in drivers:actual.update(driver.read(tick,dt).positions)
                for n in qp.names:
                    motion['travel'][n]+=abs(actual[n]-previous_joints[n])
                    motion['max_from_capture'][n]=max(motion['max_from_capture'][n],abs(actual[n]-capture_joints[n]))
                    previous_joints[n]=actual[n]
                remaining-=dt
            result['frames']+=1;previous=now;samples.append(sample(now-2.1))
            flanges=samples[-1]['flanges']
            actual_poses=([flanges[cfg[side+'_frame']] for i,side in enumerate(('left','right')) if session.active_mask&(1<<i)] if ind else
                          [flanges[cfg['left_frame']] @ qp.tool])
            target_poses=([session.target[i] for i in range(2) if session.active_mask&(1<<i)] if ind else [session.target])
            for actual_pose,target_pose in zip(actual_poses,target_poses):
                offset=np.zeros(3) if ind else session.control_point_in_tool
                error=[float(np.linalg.norm(actual_pose[:3,3]+actual_pose[:3,:3]@offset-
                    target_pose[:3,3]-target_pose[:3,:3]@offset)),
                    float(np.linalg.norm(pin.log3(actual_pose[:3,:3].T @ target_pose[:3,:3])))]
                errors.append(error)
                if not diagnostic:free_errors.append(error)
        if (clock=='received' and 'ended_by' not in result and
                segment['end']-seconds(pair[0]['stamp'])>cfg['input_max_age']):
            raise ValueError('RECORDED_INPUT_TIMEOUT')
    except (ValueError,RuntimeError) as exc:result['failure']=str(exc)
    finally:qp.close()
    # Retain accepted-motion evidence even when the recorded input ends in a
    # timeout. The original failure remains visible alongside these metrics.
    if not ind and samples:
        result['coupling']=coupling_metrics(samples,cfg,[dict(t=0.,mode=c['grasp_mode'])])
    for key,values in (('tracking',errors),('unlimited_tracking',free_errors)):
        if values:
            a=np.asarray(values)
            result[key]=dict(samples=len(a),position_rms_mm=float(np.sqrt(np.mean(a[:,0]**2)))*1000,
                position_max_mm=float(a[:,0].max())*1000,rotation_rms_deg=float(np.sqrt(np.mean(a[:,1]**2)))*180/np.pi,
                rotation_max_deg=float(a[:,1].max())*180/np.pi)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--clock',choices=['client','received'],default='client')
    p.add_argument('--runs',type=int,default=1)
    p.add_argument('--config',type=Path);p.add_argument('--profile',type=Path)
    p.add_argument('--until',type=float,help='Freeze input/event time at this UNIX timestamp')
    args=p.parse_args();until=args.until if args.until is not None else time.time()
    segments,clocks=dataset(args.log,until)
    dataset_hash=hashlib.sha256(json.dumps([s['pairs'] for s in segments],sort_keys=True).encode()).hexdigest()
    robot=RobotSetup.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/robot_snowbot_real.json')
    cfg=json.loads((args.config or robot.tool_config).read_text());cfg['urdf']=str(robot.urdf)
    if args.profile:
        from bindu_contracts.profile import Profile
        profile=Profile.load(args.profile)
    else:profile=robot.profile
    cfg['library']=str(Path(get_package_prefix('bindu_bimanual_qp'))/'lib/libbindu_bimanual_qp.so')
    results=[]
    for run in range(args.runs):
        for segment in segments:
            result=replay(segment,clocks,cfg,profile,args.clock);result['run']=run
            print(json.dumps(result,ensure_ascii=False),flush=True);results.append(result)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(schema=1,source=str(args.log),until=until,clock=args.clock,dataset_sha256=dataset_hash,
        source_poses='recorded, unsmoothed; capture anchors restored',
        scope='mapping/QP/synchronized kinematic IO only; no ROS or hardware outputs',
        config=cfg,profile=dict(max_speed=profile.max_speed,joint_dynamics=profile.joint_dynamics),
        results=results),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
