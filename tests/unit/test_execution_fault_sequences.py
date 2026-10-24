"""Seeded combinations of device faults and control fencing, with no ROS."""
from dataclasses import replace
import random
from types import SimpleNamespace as S
import unittest

from bindu_contracts.contracts import Motion
from bindu_contracts.devices import BaseReading, JointReading
from bindu_contracts.profile import Profile
from bindu_execution.executor import Executor, Rejected
from bindu_hardware.robot_io.system import RobotIO


class ExecutionFaultSequences(unittest.TestCase):
    def test_fault_orderings_preserve_stop_budget_and_revoked_authority(self):
        faults = ('stale', 'unknown_velocity', 'read_error', 'partial_write',
                  'wrong_mode', 'overrun', 'clock_reset')
        for seed in range(100):
            with self.subTest(seed=seed, fault=faults[seed % len(faults)]):
                rng = random.Random(seed)
                steady = [0.]
                state = dict(fault='', moving=False, stop_failure=False)
                writes, stops = [], []
                profile = Profile('fixture', {'left':['a'], 'right':['b']},
                    {'a':[-1.,1.], 'b':[-1.,1.]}, 1., ('joint_position',), 'fixture',
                    stop_timeout=.3)

                def read(group, now, dt):
                    if state['fault'] == 'read_error' and group == 'right':
                        raise RuntimeError('disconnected')
                    names = profile.groups[group]
                    return JointReading(10. if state['fault'] == 'stale' else now,
                        dict.fromkeys(names, 0.), state['fault'] != 'wrong_mode',
                        {} if state['fault'] == 'unknown_velocity' else
                        dict.fromkeys(names, .2 if state['moving'] else 0.))

                def write(group, target):
                    if state['fault'] == 'partial_write' and group == 'right':
                        raise RuntimeError('rejected second packet')
                    writes.append((group, dict(target)))

                def stop(group):
                    stops.append(group)
                    if state['stop_failure'] and group == 'left':
                        raise RuntimeError('stop endpoint unavailable')

                joints = {g:S(read=lambda now,dt,g=g:read(g,now,dt),
                    write_positions=lambda target,g=g:write(g,target),
                    request_stop=lambda g=g:stop(g)) for g in profile.groups}
                base = S(read=lambda now,dt:BaseReading(now,0.,0.,(0.,0.),True),
                    write_velocity=lambda velocity:None, request_stop=lambda:stop('base'))
                engine = Executor(profile, RobotIO(profile,joints,base), steady=lambda:steady[0])
                lease, epoch = engine.acquire('owner', ['left','right'], 2., 10.)
                motion = Motion('initial',lease,epoch,profile.digest,'task','','joint_target','',
                    10.,1.,names=('a','b'),positions=(.3,-.2),schema_version=2,
                    resource_groups=('left','right'),runtime_mode='simulation')
                engine.submit(motion,10.)
                now = 10.
                for _ in range(rng.randint(1,5)):
                    now += .01; steady[0] += .01; engine.tick(now)
                state.update(fault=faults[seed % len(faults)], moving=True,
                             stop_failure=bool(seed % 4 == 0))
                now += .21 if state['fault'] in ('stale','overrun') else (
                    -.01 if state['fault'] == 'clock_reset' else .01)
                steady[0] += .01
                engine.tick(now)
                if state['fault']=='partial_write':
                    events=list(engine.events)
                    detail=next(i for i,e in enumerate(events) if e.state=='DEVICE_COMMAND_DETAIL')
                    self.assertIn('PARTIAL_WRITE:',events[detail].code)
                    self.assertTrue(any(e.state in ('STOPPING','FAILED') for e in events[detail+1:]))
                self.assertFalse(engine.lease_id)
                self.assertTrue(set(stops) >= {'left','right','base'})
                deadlines = (engine.stop_deadline,engine.stop_steady_deadline)
                epoch_after_fault = engine.epoch
                count = len(writes)
                # Even restored transport cannot restore the old authority.
                state.update(fault='',stop_failure=False)
                late_rest = bool(seed % 2)
                for step in range(20):
                    now += rng.choice((0.,.01,.03))
                    steady[0] += .025
                    state['moving'] = late_rest or step < 2
                    if step % 3 == 0 and engine.stopping:
                        engine.halt(now,'REPEATED_FAULT',revoke=True,emergency=True)
                    engine.tick(now)
                    self.assertEqual((engine.stop_deadline,engine.stop_steady_deadline),deadlines)
                    self.assertEqual(engine.epoch,epoch_after_fault)
                    with self.assertRaisesRegex(Rejected,'INVALID_LEASE'):
                        engine.submit(replace(motion,command_id=f'old_{step}',stamp=now),now)
                    self.assertEqual(len(writes),count)
                self.assertIsNone(engine.active)
                self.assertIsNone(engine.stopping)
                self.assertEqual(engine.results['initial'].state,'FAILED')
                if late_rest or seed % 4 == 0:
                    self.assertTrue(engine.stop_fault)
                    state['moving'] = False
                    with self.assertRaisesRegex(Rejected,'STOP_UNCONFIRMED'):
                        engine.acquire('new',['left'],2.,now+.01)


if __name__ == '__main__':
    unittest.main()
