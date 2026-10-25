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
from dataclasses import fields, replace
import gc
import hashlib
import json
import math
from pathlib import Path
import time

from bindu_contracts.teleoperation import VRFrame


def records(path, *, strict=False):
    chunks=[path]+sorted(path.parent.glob(path.stem+'-[0-9][0-9][0-9][0-9]'+path.suffix))
    for chunk_index,chunk in enumerate(chunks):
        if strict and (not chunk.is_file() or chunk_index and
                       chunk.stem!=f'{path.stem}-{chunk_index:04d}'):
            raise ValueError('REPLAY_MISSING_CHUNK:'+chunk.name)
        with chunk.open() as stream:
            for index,line in enumerate(stream,1):
                try:yield json.loads(line)['data']
                except (ValueError,KeyError,TypeError) as exc:
                    if strict:raise ValueError(f'REPLAY_CORRUPT_RECORD:{chunk.name}:{index}') from exc
                    continue  # Legacy diagnostic files may have an incomplete last line.


def seconds(stamp):
    return stamp['sec']+stamp['nanosec']*1e-9 if isinstance(stamp,dict) else float(stamp)


def frame(d, stamp):
    # One fixed clock offset preserves head age relative to the hand. Never
    # replace head sample time with the replay delivery time or zero buttons.
    values={f.name:d[f.name] for f in fields(VRFrame) if f.name in d}
    values['stamp']=stamp
    values.setdefault('mapping','pico_snowbot_v1')
    for key in ('pose','button_edges','stick','head_pose'):
        if key in values:values[key]=tuple(values[key])
    if values.get('head_pose'):
        values['head_stamp']=seconds(d['head_stamp'])+stamp-seconds(d['stamp'])
    return VRFrame(**values)


def input_audit(folder, *, require_full=False):
    """Audit observer capture, without importing ROS, a solver, or a publisher."""
    health_path=folder/'diagnostics-health.json'
    health=json.loads(health_path.read_text()) if health_path.is_file() else {}
    capture=health.get('input_recording',{})
    full='vr-raw' in capture.get('full_streams',[])
    if require_full and not full:raise ValueError('REPLAY_FULL_INPUTS_REQUIRED')
    if full and (not capture.get('closed') or health.get('error') or
                 any(capture.get('dropped',{}).get(s,0) for s in capture['full_streams'])):
        raise ValueError('REPLAY_INPUT_CAPTURE_INCOMPLETE')
    pairs={};last={};gaps=[];count=0
    for d in records(folder/'vr-raw.jsonl',strict=full):
        key=(d['source_id'],d['seq']);side=d['side'];count+=1
        if side not in ('left','right'):raise ValueError('REPLAY_INPUT_SIDE')
        pair=pairs.setdefault(key,set())
        if side in pair:raise ValueError('REPLAY_DUPLICATE_INPUT')
        pair.add(side)
        previous=last.get((key[0],side));at=seconds(d['stamp'])
        if not math.isfinite(at):raise ValueError('REPLAY_INPUT_STAMP')
        if previous:
            seq,stamp=previous
            if key[1]<=seq or at<stamp:raise ValueError('REPLAY_INPUT_ORDER')
            if key[1]>seq+1:gaps.append(dict(source_id=key[0],side=side,after_seq=seq,
                before_seq=key[1],missing=key[1]-seq-1,interval_s=at-stamp))
        last[(key[0],side)]=(key[1],at)
    incomplete=sum(pair!={'left','right'} for pair in pairs.values())
    if not count:raise ValueError('REPLAY_NO_INPUT')
    if full:
        written=capture.get('written',{}).get('vr-raw',0)
        if count!=written or count!=capture.get('received',{}).get('vr-raw',0) or incomplete:
            raise ValueError('REPLAY_INPUT_CAPTURE_COUNT_OR_PAIR')
    return dict(mode='all_observer_received' if full else 'legacy_sampled_or_unknown',
        messages=count,paired_events=len(pairs)-incomplete,incomplete_pairs=incomplete,
        sources=sorted({source for source,_ in pairs}),sequence_gaps=gaps,
        upstream_complete=False,scope='receiver coalescing/DDS loss cannot be excluded; no gap filling')


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
        candidates=[k for k in pairs if k[1]==seq and
                    (not c.get('input_source_id') or k[0]==c['input_source_id']) and
                    abs(seconds(pairs[k][0]['stamp'])-c['input_stamp'])<=.05]
        if not candidates:raise ValueError('CAPTURE_INPUT_MISSING')
        if len(candidates)!=1:raise ValueError('CAPTURE_SOURCE_AMBIGUOUS')
        key=candidates[0]
        segment['pairs']=[pairs[k] for k in sorted(pairs,key=lambda k:seconds(pairs[k][0]['stamp']))
            if k[0]==key[0] and k[1]>=seq and seconds(pairs[k][0]['stamp'])<=segment['end']]
    return segments,clocks


def replay(segment,clocks,cfg,profile,clock):
    import numpy as np
    from bindu_hardware.drivers.simulated_joints import SimJointDriver, SynchronizedJointProgress
    from bindu_kinematics.bimanual import BimanualQP
    from bindu_kinematics.tool_geometry import make_collision_checker, coupling_metrics
    from bindu_teleoperation.bimanual_session import BimanualSession
    from bindu_teleoperation.independent_session import IndependentArmsSession
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
        actual=dict(c['feedback']);captured=c.get('capture_reference',actual)
        reference=[captured[n] for n in qp.names]
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
        for at,seq,released in ((1.,0,True),(1.02,1,False),(2.1,2,False)):
            pair=[replace(frame(d,at),seq=seq,grip=0. if released else d['grip'],
                          virtual_grip=c['grasp_mode']=='right_axial') for d in first]
            session.ingest(*pair,at)
        home=qp.capture_independent(captured) if ind else qp.capture(captured,c['grasp_mode'])
        session.capture(home)
        yaw=c.get('latched_operator_yaw_rad')
        if yaw is not None:
            if ind:
                for hand,value in zip(session.hands,yaw):hand.mapping_yaw=value
            else:session.mapping_yaw=yaw
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
            if index==0:continue  # The recorded capture frame is the restored baseline.
            if now<previous:raise ValueError('REPLAY_CLOCK_REVERSED')
            gap=now-previous
            if gap>cfg['input_max_age']:raise ValueError('RECORDED_INPUT_TIMEOUT')
            frames=[replace(frame(d,now),virtual_grip=c['grasp_mode']=='right_axial') for d in pair]
            if any(f.stop for f in frames):
                result['ended_by']='RECORDED_OPERATOR_STOP';break
            session.ingest(*frames,now)
            if session.mode!='follow' or session.grasp_mode!=last_mask:
                result['ended_by']='CLUTCH_OR_GRASP_TRANSITION';break
            if gap==0:continue  # Same-time button changes were still ingested.
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
    p.add_argument('--clock',choices=['client','received'],default='received',
                   help='received preserves source reception gaps; client is a separate jitter-removal experiment')
    p.add_argument('--require-full-inputs',action='store_true',help='Reject sampled, unclosed or lossy observer captures')
    p.add_argument('--inspect-inputs',action='store_true',help='Audit input only, without ROS or QP dependencies')
    p.add_argument('--runs',type=int,default=1)
    p.add_argument('--config',type=Path);p.add_argument('--profile',type=Path)
    p.add_argument('--until',type=float,help='Freeze input/event time at this UNIX timestamp')
    args=p.parse_args();until=args.until if args.until is not None else time.time()
    if args.runs<1 or not math.isfinite(until):p.error('runs must be positive and until must be finite')
    audit=input_audit(args.log,require_full=args.require_full_inputs)
    if args.inspect_inputs:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(audit,ensure_ascii=False));return
    from ament_index_python.packages import get_package_prefix
    from bindu_runtime.robot_setup import RobotSetup
    segments,clocks=dataset(args.log,until)
    if not segments:raise ValueError('REPLAY_NO_CAPTURED_SEGMENTS')
    dataset_hash=hashlib.sha256(json.dumps(dict(segments=segments,
        clocks=sorted((source,seq,at) for (source,seq),at in clocks.items()) if args.clock=='client' else []),
        sort_keys=True).encode()).hexdigest()
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
        input_audit=audit,replay_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_poses='recorded, unsmoothed; capture anchors restored',
        scope='mapping/QP/synchronized kinematic IO only; no ROS or hardware outputs',
        config=cfg,profile=dict(max_speed=profile.max_speed,joint_dynamics=profile.joint_dynamics),
        results=results),ensure_ascii=False,indent=2)+'\n')
    return int(any(result.get('failure') for result in results))


if __name__=='__main__':raise SystemExit(main())
