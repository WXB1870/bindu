"""Long-running physical-stream checks with bounded observation buffers."""
import gzip
import json
import math
from collections import deque
from pathlib import Path
import time
import rclpy
from bindu_interfaces.msg import ExecutionState, MotionCommand
from bindu_interfaces.srv import ControlExecution
from bindu_runtime.common import EVENT_QOS
from validate_ros import until, wait


def process_memory(root_pid):
    pending=[root_pid];out={}
    while pending:
        pid=pending.pop();root=Path('/proc')/str(pid)
        try:
            pending.extend(int(x) for x in (root/'task'/str(pid)/'children').read_text().split())
            args=(root/'cmdline').read_bytes().split(b'\0')
            role=next((Path(a.decode()).name for a in args if a.endswith((b'/execution',b'/recorder'))),None)
            if role:
                status=dict(line.split(':',1) for line in (root/'status').read_text().splitlines() if ':' in line)
                stat=(root/'stat').read_text().split()
                out[role]={'pid':pid,'rss_kib':int(status['VmRSS'].split()[0]),'cpu_ticks':int(stat[13])+int(stat[14])}
        except (OSError,KeyError,ValueError):pass
    return out


def soak_checks(node,args,profile,seen,results,command,release,drain,control,process):
    import numpy as np
    seconds=args.soak_seconds
    assert math.isfinite(seconds) and seconds>=10
    amplitude,period=args.soak_amplitude,args.soak_period
    assert math.isfinite(amplitude) and amplitude>0 and math.isfinite(period) and period>0
    count=round(seconds*100);acks=bytearray(count);latencies=[];intervals=[];memory=[]
    errors=[];reference=deque(maxlen=128);max_error=0.;duplicates=0
    sent=0;last_send=None;started=None;previous_accept=None;max_ack_gap=0.
    base=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
    cmd=command('left_arm','joint_target');cmd.joint_names=profile.groups['left_arm'];cmd.valid_for=.25
    prefix=cmd.command_id+'_soak_';names=cmd.joint_names
    minimum=[base[n] for n in names];maximum=minimum.copy()
    for j,n in enumerate(names):
        excursion=amplitude if j==0 else amplitude*.6 if j==3 else 0.
        assert profile.limits[n][0]<=base[n]-excursion<=base[n]+excursion<=profile.limits[n][1],n
        assert excursion*2*math.pi/period<profile.speed(n),n
    publisher=node.create_publisher(MotionCommand,args.namespace+'/execution/targets',8)
    telemetry=gzip.open(args.output/'soak-execution.jsonl.gz','wt')
    commands=gzip.open(args.output/'soak-inputs.jsonl.gz','wt')
    def accepted(msg):
        nonlocal duplicates,previous_accept,max_ack_gap
        if msg.command_id.startswith(prefix):
            i=int(msg.command_id[len(prefix):]);duplicates+=bool(acks[i]);acks[i]=1
            now=node.get_clock().now().nanoseconds/1e9
            latencies.append(now-msg.stamp.sec-msg.stamp.nanosec/1e9)
            if previous_accept is not None:max_ack_gap=max(max_ack_gap,now-previous_accept)
            previous_accept=now
    def state(msg):
        nonlocal max_error
        for i,n in enumerate(names):
            if n in msg.joints.name:
                q=msg.joints.position[list(msg.joints.name).index(n)]
                minimum[i]=min(minimum[i],q);maximum[i]=max(maximum[i],q)
        t=msg.stamp.sec+msg.stamp.nanosec/1e9;ft=msg.feedback_stamp.sec+msg.feedback_stamp.nanosec/1e9
        if all(n in msg.reference.name for n in names):
            reference.append((t,[msg.reference.position[list(msg.reference.name).index(n)] for n in names]))
            if len(reference)>1 and reference[0][0]<=ft<=reference[-1][0]:
                for i in range(len(reference)-1):
                    a,b=reference[i],reference[i+1]
                    if a[0]<=ft<=b[0]:
                        ratio=(ft-a[0])/max(b[0]-a[0],1e-9)
                        e=[msg.joints.position[list(msg.joints.name).index(n)]-(x+(y-x)*ratio)
                           for n,x,y in zip(names,a[1],b[1])]
                        errors.append(sum(v*v for v in e)/len(e));max_error=max(max_error,max(map(abs,e)))
                        break
        telemetry.write(json.dumps({'stamp':t,'feedback_stamp':ft,'state':msg.state,'code':msg.code,
            'command_id':msg.command_id,'names':list(msg.joints.name),'q':list(msg.joints.position),
            'reference_names':list(msg.reference.name),'reference':list(msg.reference.position),
            'dq':list(msg.reference.velocity),'ddq':list(msg.reference_accelerations),'jerk':list(msg.reference_jerks)})+'\n')
    sub=node.create_subscription(MotionCommand,args.namespace+'/execution/accepted',accepted,EVENT_QOS)
    st=node.create_subscription(ExecutionState,args.namespace+'/execution/state',state,100)
    until(node,lambda:publisher.get_subscription_count()>0)
    try:
        started=time.monotonic();next_report=started
        for i in range(count):
            while time.monotonic()<started+i*.01:rclpy.spin_once(node,timeout_sec=.001)
            now=time.monotonic()
            if last_send is not None:intervals.append(now-last_send)
            last_send=now
            cmd.command_id=prefix+str(i);cmd.stamp=node.get_clock().now().to_msg()
            phase=i*.01*2*math.pi/period
            cmd.positions=[base[n]+(amplitude if j==0 else -amplitude*.6 if j==3 else 0.)*math.sin(phase) for j,n in enumerate(names)]
            publisher.publish(cmd);sent+=1
            commands.write(json.dumps({'stamp':cmd.stamp.sec+cmd.stamp.nanosec/1e9,'sequence':i,'positions':list(cmd.positions)})+'\n')
            rclpy.spin_once(node,timeout_sec=0.)
            status=seen['state']
            assert not (status.command_id.startswith(prefix) and status.state in ('FAILED','CANCELED','STOPPING')),(i,status.state,status.code)
            assert max_error<.025,('tracking',max_error)
            if now>=next_report:
                row={'elapsed':now-started,'sent':sent,'accepted':sum(acks),'memory':process_memory(process.pid),
                     'tracking_max_rad':max_error,'ack_latency_p95_ms':float(np.percentile(latencies,95)*1000) if latencies else None}
                memory.append(row);(args.output/'soak-progress.json').write_text(json.dumps(row,indent=2)+'\n')
                commands.flush();telemetry.flush();next_report=now+10
        until(node,lambda:sum(acks)==count,seconds=3.)
        assert not duplicates,duplicates
        assert len(errors)>count*.5,('tracking_sample_count',len(errors),count)
        measured_range=[b-a for a,b in zip(minimum,maximum)]
        if seconds>=period:
            assert measured_range[0]>amplitude*1.8 and measured_range[3]>amplitude*.6*1.8,measured_range
        response=wait(node,control.call_async(ControlExecution.Request(operation='stop',lease_id=cmd.lease_id,epoch=cmd.epoch)))
        assert response.ok,response.code
        until(node,lambda:not seen['state'].reference.name and seen['state'].state!='STOPPING',seconds=6.)
        drain(.5);release()
        rss=[r['memory']['execution']['rss_kib'] for r in memory if r['elapsed']>=60 and 'execution' in r['memory']]
        growth=(sum(rss[-3:])/len(rss[-3:])-sum(rss[:3])/len(rss[:3]))/1024 if len(rss)>=6 else None
        assert growth is None or growth<20,('executor_rss_growth_mib',growth)
        results.append({'case':'physical_stream_soak','passed':True,'duration_seconds':seconds,'sent':sent,'accepted':sum(acks),
            'shoulder_amplitude_rad':amplitude,'period_seconds':period,
            'mean_hz':(sent-1)/sum(intervals),'input_interval_p95_ms':float(np.percentile(intervals,95)*1000),
            'accept_latency_p95_ms':float(np.percentile(latencies,95)*1000),'max_accept_gap_ms':max_ack_gap*1000,
            'tracking_rms_rad':math.sqrt(sum(errors)/len(errors)),'tracking_max_rad':max_error,
            'tracking_samples':len(errors),'measured_joint_range_rad':dict(zip(names,measured_range)),
            'executor_rss_growth_mib_after_warmup':growth})
    finally:
        telemetry.close();commands.close();node.destroy_subscription(sub);node.destroy_subscription(st);node.destroy_publisher(publisher)
        (args.output/'soak-resources.json').write_text(json.dumps(memory,indent=2)+'\n')
        (args.output/'soak-counts.json').write_text(json.dumps({'sent':sent,'accepted':sum(acks),'duplicates':duplicates,
            'unacknowledged_indices':[i for i in range(sent) if not acks[i]]})+'\n')
