#!/usr/bin/env python3
"""Test-only remote Pi peer. No inference; explicit synthetic joint targets.

Uses independent JSON framing so compatibility is not tested by echoing our codec.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import zmq


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--fault',default='')
    parser.add_argument('--correlate',action='store_true')
    args=parser.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    context=zmq.Context()
    commands=context.socket(zmq.PUB); commands.setsockopt(zmq.LINGER,0)
    commands.bind(cfg['command_connect'])
    state=context.socket(zmq.SUB if cfg['mode']=='pubsub' else zmq.REQ)
    state.setsockopt(zmq.LINGER,0)
    if cfg['mode']=='pubsub': state.setsockopt_string(zmq.SUBSCRIBE,cfg['state_topic'])
    state.connect(cfg['state_bind'])
    observations=0; sequence=0; waiting=False; latest=None; first_command=None
    started=time.monotonic(); next_command=0.; next_request=0.
    try:
        while time.monotonic()-started<15:
            now=time.monotonic()
            if cfg['mode']=='pull' and not waiting and now>=next_request:
                state.send_multipart([cfg['request_topic'].encode(),json.dumps({'seq':observations,'timestamp_send':time.time()}).encode(),b'{}'])
                waiting=True;next_request=now+.04
            if state.poll(5,zmq.POLLIN):
                frames=state.recv_multipart();waiting=False
                assert frames[0].decode()==cfg['state_topic']
                header=json.loads(frames[1]);payload=json.loads(frames[2])
                if payload.get('ok',True):
                    assert payload['prompt']=='pick water'
                    assert set(payload['关节状态字典'])==set(cfg['joint_map'].values())
                    assert len(header['arrays'])==3
                    for meta,raw in zip(header['arrays'],frames[3:]):
                        assert meta['dtype']=='uint8' and meta['shape']==[224,224,3]
                        assert len(raw)==224*224*3
                    latest=payload;observations+=1
                    Path(args.output).write_text(json.dumps({'observations':observations,'mode':cfg['mode'],
                        'synthetic':True,'prompt':payload['prompt'],'last_observation_id':payload['bindu']['observation_id']}))
            if latest and now>=next_command:
                first_command=first_command or now
                if args.fault=='disconnect' and now-first_command>.6:
                    continue
                target={wire:.2 for wire in cfg['joint_map'].values()}
                if args.fault=='missing': target.pop('left_joint_2')
                payload={'关节命令字典':target}
                if args.correlate:
                    payload['bindu']=dict(latest['bindu'])
                    if args.fault=='wrong_session':payload['bindu']['session_id']='previous-session'
                timestamp=time.time()-(3. if args.fault=='stale' else 0.)
                header={'topic':cfg['command_topic'],'seq':sequence,'timestamp_send':timestamp,'encoding':'json','arrays':[]}
                frames=[cfg['command_topic'].encode(),json.dumps(header).encode(),json.dumps(payload,ensure_ascii=False).encode()]
                if args.fault=='malformed':frames[1]=b'not-json'
                commands.send_multipart(frames)
                sequence+=1;next_command=now+.02
    except KeyboardInterrupt:
        pass
    finally:
        state.close();commands.close();context.term()


if __name__=='__main__':main()
