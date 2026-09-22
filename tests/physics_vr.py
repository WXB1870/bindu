"""Synthetic hand-controller motion through real Vuer, IK and PhysX.

No direct VRInput or joint stream is published by the operator peer. Only the
explicit initial bent-arm fixture uses the motion service before VR takes over.
"""
import gzip
import json
import math
import threading
import time
import uuid
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from bindu_interfaces.action import TeleopSession
from bindu_interfaces.msg import VRInput, RuntimeEvent, MotionCommand, ExecutionState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
from validate_ros import until, wait


def rotation_x(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])


class ControllerPeer:
    def __init__(self,args):
        self.args=args;self.stop=threading.Event();self.errors=[]
        self.control={'grip':0.,'enabled':True,'valid':True,'origin':np.array([-.25 if args.side=='left' else .25,1.2,-.35]),
                      'rotation':np.eye(3),'motion_start':None,'duration':10.}
        self.sent=0;self.display_seen=False;self.markers_seen=False;self.intervals=[]
        self.thread=threading.Thread(target=self.run,daemon=True)

    def run(self):
        from websockets.sync.client import connect
        from msgpack import packb,unpackb
        rng=np.random.default_rng(20260922);basis=OPENXR_TO_ROBOT[:3,:3]
        log=gzip.open(self.args.output/'vr-wire.jsonl.gz','wt')
        try:
            end=time.monotonic()+20;ws=None
            while ws is None and time.monotonic()<end and not self.stop.is_set():
                try:ws=connect(f'ws://127.0.0.1:{self.args.vr_port}/',open_timeout=1)
                except OSError:self.stop.wait(.1)
            if ws is None:raise RuntimeError('Vuer websocket not available')
            previous=None;deadline=time.monotonic()
            with ws:
                while not self.stop.is_set():
                    state=self.control.copy();now=time.monotonic()
                    if state['enabled']:
                        phase=0. if state['motion_start'] is None else min(1.,max(0.,(now-state['motion_start'])/state['duration']))*2*math.pi
                        sign=1 if self.args.side=='left' else -1
                        delta=np.array([.04*math.sin(phase),-sign*.08*(1-math.cos(phase))/2,.06*math.sin(phase)])
                        noise=rng.normal(0.,.00015,3) if state['motion_start'] is not None else np.zeros(3)
                        pose=np.eye(4);pose[:3,3]=state['origin']+basis.T@(delta+noise)
                        pose[:3,:3]=basis.T@rotation_x(.08*math.sin(phase))@basis@state['rotation']
                        neutral=np.eye(4);neutral[:3,3]=[.25 if self.args.side=='left' else -.25,1.2,-.35]
                        buttons={'squeezeValue':state['grip'],'triggerValue':.25+.1*math.sin(phase),'aButton':False,'bButton':False}
                        other='right' if self.args.side=='left' else 'left'
                        value={self.args.side:pose.ravel(order='F').tolist() if state['valid'] else None,
                            other:neutral.ravel(order='F').tolist(),self.args.side+'State':buttons,
                            other+'State':dict(buttons,squeezeValue=0.,triggerValue=0.)}
                        event={'etype':'CONTROLLER_MOVE','key':'motionControllers','value':value}
                        tx=time.time();ws.send(packb(event,use_bin_type=True));self.sent+=1
                        if previous is not None:self.intervals.append(now-previous)
                        previous=now
                        log.write(json.dumps({'tx_stamp':tx,'sequence':self.sent,'event':event})+'\n')
                    else:previous=None
                    for _ in range(3):
                        try:
                            packet=repr(unpackb(ws.recv(timeout=.0001),raw=False))
                            self.display_seen|='teleopStatus' in packet
                            self.markers_seen|='targetPose' in packet and 'measuredPose' in packet
                        except TimeoutError:break
                    # Seeded +/-2 ms interval jitter, with a periodic 25 ms
                    # scheduling delay; no synthetic refresh of source stamps.
                    deadline+=1/72+rng.uniform(-.002,.002)+(.025 if self.sent and self.sent%180==0 else 0.)
                    self.stop.wait(max(0.,deadline-time.monotonic()))
                    if time.monotonic()-deadline>.05:deadline=time.monotonic()
        except Exception as exc:self.errors.append(repr(exc))
        finally:log.close()

    def close(self):
        self.stop.set();self.thread.join(timeout=4)
        assert not self.thread.is_alive(),'VR peer failed to stop'


def vr_checks(node,args,profile,seen,results,command,send,finished,release,drain):
    peer=ControllerPeer(args);events=[];accepted=[];frames=[];views=[];states=[];subs=[]
    log=gzip.open(args.output/'vr-chain.jsonl.gz','wt');active_goal=None
    def capture(kind,rows):
        def receive(msg):
            rows.append(msg)
            log.write(json.dumps({'received_stamp':node.get_clock().now().nanoseconds/1e9,
                'kind':kind,'message':message_to_ordereddict(msg)})+'\n')
        return receive
    for message,topic,kind,rows in [(RuntimeEvent,'events','event',events),(MotionCommand,'execution/accepted','accepted',accepted),
            (VRInput,'teleop/vr/input','input',frames),(ExecutionState,'execution/state','execution',states)]:
        subs.append(node.create_subscription(message,args.namespace+'/'+topic,capture(kind,rows),100))
    subs.append(node.create_subscription(String,args.namespace+'/teleop/display',lambda m:views.append(json.loads(m.data)),
        QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)))
    client=ActionClient(node,TeleopSession,args.namespace+'/teleop/session')
    ready=node.create_client(Trigger,args.namespace+'/teleop/ready')
    names=profile.groups[args.side+'_arm'];run=uuid.uuid4().hex
    def q():return dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
    def ramp(grip):
        old=peer.control['grip']
        for step in range(1,7):peer.control['grip']=old+(grip-old)*step/6;drain(.04)
    def start_session(label):
        nonlocal active_goal
        peer.control.update(grip=0.,motion_start=None,valid=True,enabled=True)
        drain(.3)
        reply=wait(node,ready.call_async(Trigger.Request()));assert reply.success,reply.message
        active_goal=wait(node,client.send_goal_async(TeleopSession.Goal(task_id=run+'_'+label,resource_group=args.side+'_arm',duration=90.)))
        assert active_goal.accepted,'VR session rejected'
        future=active_goal.get_result_async();drain(.2);count=len(accepted)
        ramp(1.)
        until(node,lambda:len(accepted)>=count+5 or future.done(),seconds=5.)
        assert not future.done(),future.result()
        return future
    def operate(seconds,future):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            drain(.03)
            assert not future.done(),future.result()
            assert not peer.errors,peer.errors
    def held():
        until(node,lambda:not seen['state'].reference.name and seen['state'].state!='STOPPING',seconds=profile.stop_timeout+1)
        drain(.3);before=q();count=len(accepted);drain(.5)
        drift=max(abs(q()[n]-before[n]) for n in names);speed=max(map(abs,seen['physics'].joints.velocity))
        assert drift<.003 and speed<.02,(drift,speed)
        assert len(accepted)==count,'late target after confirmed hold'
        return drift,speed
    try:
        # Finish Vuer/IK/observer startup before requesting physical motion.
        # Their readiness is independent of the executor's service discovery.
        peer.thread.start()
        assert client.wait_for_server(timeout_sec=15) and ready.wait_for_service(timeout_sec=10)
        until(node,lambda:frames and views and views[-1]['measured'] is not None,seconds=25.)
        deadline=time.monotonic()+15
        while True:
            reply=wait(node,ready.call_async(Trigger.Request()))
            if reply.success:break
            assert time.monotonic()<deadline,reply.message
            drain(.1)
        # Service readiness alone does not establish stable sampling during
        # Isaac/ROS discovery. Require five seconds of fresh physics before
        # the setup move, without relaxing any running-session timeout.
        stable_since=time.monotonic();warmup_deadline=stable_since+20
        while time.monotonic()-stable_since<5:
            drain(.02)
            stamp=seen['physics'].stamp
            if node.get_clock().now().nanoseconds/1e9-stamp.sec-stamp.nanosec/1e9>.05:
                stable_since=time.monotonic()
            assert time.monotonic()<warmup_deadline,'physics startup did not stabilize'
        # This setup is a simulator fixture, not a VR-generated command.
        cmd=command(args.side+'_arm','finite_trajectory');cmd.joint_names=names
        target=[0.]*7;target[0]=.45 if args.side=='left' else -.45;target[3]=-.3 if args.side=='left' else .3
        cmd.points=[JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=4))]
        send(cmd);finished(cmd);release();drain(.3);accepted.clear();states.clear()
        future=start_session('motion_clutch');begin=len(views)
        peer.control.update(motion_start=time.monotonic(),duration=20.)
        operate(20.2,future)
        motion_views=[v for v in views[begin:] if v.get('error')]
        assert len(motion_views)>40,len(motion_views)
        errors=[v['error']['position_m'] for v in motion_views]
        assert max(errors)<.03,max(errors)
        # Release while moving, then relocate the hand and rotate the wrist.
        peer.control.update(motion_start=time.monotonic(),duration=20.);operate(1.,future);ramp(0.)
        drift,speed=held();count=len(accepted);hold_q=q()
        peer.control.update(motion_start=None,origin=peer.control['origin']+np.array([.18,.06,-.10]),rotation=rotation_x(.5))
        operate(.5,future);assert len(accepted)==count
        ramp(1.);until(node,lambda:len(accepted)>count or future.done(),seconds=5.)
        assert not future.done(),future.result()
        anchor=accepted[count];jump=max(abs(v-hold_q[n]) for n,v in zip(anchor.joint_names,anchor.positions))
        assert jump<.005,jump
        peer.control.update(motion_start=time.monotonic(),duration=20.);operate(20.2,future)
        assert wait(node,active_goal.cancel_goal_async()).goals_canceling
        result=wait(node,future,seconds=8.);assert result.status==5 and result.result.code=='CANCELED',result
        active_goal=None;held()
        results.append({'case':'vr_continuous_motion_clutch_reanchor_cancel','passed':True,'side':args.side,
            'paused_drift_rad':drift,'paused_speed_rad_s':speed,'reanchor_max_joint_jump_rad':jump,
            'hand_relocation_m':math.sqrt(.18**2+.06**2+.10**2),'wrist_relocation_rad':.5,
            'display_samples_with_tracking_error':len(motion_views),
            'observed_tracking_rms_m':float(np.sqrt(np.mean(np.square(errors)))),
            'observed_tracking_max_m':max(errors),
            'tracking_measurement':'10 Hz observer: latest target vs FK of actual PhysX joints; includes timing lag'})
        for fault,expected in [('tracking','VR_TRACKING_INVALID'),('silence','VR_INPUT_TIMEOUT')]:
            future=start_session(fault);peer.control.update(motion_start=time.monotonic(),duration=20.)
            operate(1.5,future);at=time.monotonic()
            if fault=='tracking':peer.control['valid']=False
            else:peer.control['enabled']=False
            result=wait(node,future,seconds=8.);elapsed=time.monotonic()-at
            assert result.status==6 and result.result.code==expected,result
            active_goal=None;drift,speed=held();count=len(accepted)
            peer.control.update(valid=True,enabled=True,grip=0.,motion_start=None)
            drain(.6);assert len(accepted)==count,'input return resumed old session'
            results.append({'case':'vr_'+fault+'_physical_stop','passed':True,'code':result.result.code,
                'result_after_fault_seconds':elapsed,'hold_drift_rad':drift,'max_speed_rad_s':speed,
                'automatic_resume':False})
        assert peer.display_seen and peer.markers_seen,'no Vuer feedback rendered'
        assert frames and all(m.source_id and m.side==args.side for m in frames)
        assert any(not m.valid for m in frames),'tracking loss did not traverse decoder'
        assert all(c.resource_group==args.side+'_arm' and c.joint_names==names and c.observation_id==c.command_id for c in accepted)
        positions=[dict(zip(s.joints.name,s.joints.position)) for s in states if s.joints.name]
        ranges={n:max(p[n] for p in positions)-min(p[n] for p in positions) for n in names}
        assert max(ranges.values())>.05,ranges
        results.append({'case':'vr_wire_chain_evidence','passed':True,'wire_frames':peer.sent,'decoded_frames':len(frames),
            'accepted_targets':len(accepted),'input_interval_p95_ms':float(np.percentile(peer.intervals,95)*1000),
            'measured_joint_range_rad':ranges,'vuer_feedback_packets_received':True,'wire_format':'v3.4 CONTROLLER_MOVE / column-major OpenXR',
            'input_origin':'synthetic hand poses; real Vuer WebSocket receiver, no headset',
            'nominal_rate_hz':72,'translation_noise_std_m':.00015,'interval_jitter_seconds':.002})
    finally:
        if active_goal is not None:
            try:wait(node,active_goal.cancel_goal_async(),seconds=2.)
            except Exception:pass
        if peer.thread.ident is not None:peer.close()
        log.close()
        (args.output/'vr-display.json').write_text(json.dumps(views)+'\n')
        (args.output/'vr-peer-errors.json').write_text(json.dumps(peer.errors)+'\n')
        for sub in subs:node.destroy_subscription(sub)
        client.destroy();node.destroy_client(ready)
