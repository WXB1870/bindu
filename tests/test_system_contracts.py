import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
from bindu_contracts.profile import Profile
from bindu_contracts.contracts import Motion, Observation
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_hand import SimHandDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO
from bindu_tasks.scene import Scene
from bindu_execution.executor import Executor, Rejected
from bindu_recording.recorder import AsyncRecorder


class Contracts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.profile_path = self.path/'profile.json'
        self.profile_path.write_text(json.dumps(dict(name='test',groups={'arm':['a','b'],'hand':['f']},
            limits={n:[-1,1] for n in ['a','b','f']},max_speed=1.5,
            capabilities=['joint_position','base_velocity','simulated_grasp'])))
        self.p = Profile.load(self.profile_path)
        self.drivers = {'arm': SimJointDriver(self.p.groups['arm'], self.p.max_speed),
                        'hand': SimHandDriver(self.p.groups['hand'], self.p.max_speed)}
        self.base = SimBaseDriver()
        self.io = RobotIO(self.p, self.drivers, self.base)
        self.e = Executor(self.p, self.io)
        lease, epoch = self.e.acquire('test',['arm','hand','base'],2.,10.)
        self.m = Motion('cmd',lease,epoch,self.p.digest,'task','obs','finite_trajectory','arm',10.,1.5,
                        names=('a','b'),offsets=(.5,),points=((.3,.3),))

    def test_feedback_completion(self):
        self.e.submit(self.m,10.)
        self.e.tick(10.5)  # Publishing the final reference alone cannot finish.
        self.assertIsNotNone(self.e.active)
        for i in range(51,101): self.e.tick(10+i/100)
        self.assertEqual(self.e.results['cmd'].state,'SUCCEEDED')
        self.assertAlmostEqual(self.e.feedback.positions['a'],.3)

    def test_exclusive_and_epoch_fencing(self):
        with self.assertRaisesRegex(Rejected,'BUSY'): self.e.acquire('other',['hand'],2.,10.)
        self.e.halt(10.,revoke=True)
        self.e.acquire('new',['arm'],2.,10.)
        with self.assertRaisesRegex(Rejected,'INVALID_LEASE'): self.e.submit(self.m,10.)

    def test_invalid_inputs(self):
        cases = [dict(names=('b','a')),dict(points=((float('nan'),0.),)),dict(points=((2.,0.),)),
                 dict(offsets=(0.,)),dict(offsets=(.01,)),dict(profile_hash='bad'),
                 dict(stamp=9.,valid_for=.5),dict(positions=(0.,0.)),dict(velocity=(.1,0.))]
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(Rejected):
                self.e.submit(replace(self.m,**changes),10.)

    def test_online_expiry(self):
        m=replace(self.m,mode='joint_target',positions=(.3,.3),points=(),offsets=(),valid_for=.2)
        self.e.submit(m,10.)
        self.e.tick(10.1)
        self.e.tick(10.21)
        self.assertEqual(self.e.results['cmd'].code,'COMMAND_TIMEOUT')
        self.assertEqual(self.drivers['arm'].target,self.drivers['arm'].positions)
        self.assertFalse(self.io.reference_positions)

    def test_future_wait(self):
        m=replace(self.m,mode='joint_target',stamp=10.04,positions=(.3,.3),points=(),offsets=())
        self.e.submit(m,10.)
        self.e.tick(10.02)
        self.assertEqual(self.drivers['arm'].target['a'],0.)
        self.e.tick(10.05)
        self.assertEqual(self.drivers['arm'].target['a'],.3)

    def test_replacement(self):
        self.e.submit(self.m,10.)
        self.e.submit(replace(self.m,command_id='cmd2'),10.)
        self.assertEqual(self.e.results['cmd'].state,'SUPERSEDED')
        with self.assertRaisesRegex(Rejected,'DUPLICATE'): self.e.submit(self.m,10.)

    def test_lease_expiry(self):
        self.e.submit(replace(self.m,valid_for=5.),10.)
        self.e.tick(12.1)
        self.assertEqual(self.e.results['cmd'].code,'LEASE_EXPIRED')
        self.assertFalse(self.e.lease_id)

    def test_clock_reset(self):
        self.e.submit(self.m,10.)
        self.e.tick(9.)
        self.assertEqual(self.e.results['cmd'].code,'CLOCK_RESET')
        self.assertFalse(self.e.lease_id)

    def test_feedback_loss(self):
        self.e.submit(self.m,10.)
        self.drivers['hand'].inject_fault('feedback_loss')
        self.e.tick(10.3)
        self.assertEqual(self.e.results['cmd'].code,'FEEDBACK_STALE')
        self.assertFalse(self.e.lease_id)

    def test_backend_reject(self):
        self.e.submit(self.m,10.)
        self.drivers['arm'].inject_fault('reject')
        self.e.tick(10.01)
        self.assertEqual(self.e.results['cmd'].code,'DEVICE_COMMAND_REJECTED')

    def test_base_path(self):
        m=replace(self.m,mode='base_velocity',group='base',names=(),offsets=(),points=(),velocity=(.2,0.),duration=.3)
        self.e.submit(m,10.)
        for i in range(1,51): self.e.tick(10+i/100)
        self.assertEqual(self.e.results['cmd'].state,'SUCCEEDED')
        self.assertGreater(self.e.feedback.base_x,.04)
        self.assertEqual(self.e.feedback.base_velocity,(0.,0.))

    def test_scene_evidence(self):
        s=Scene()
        with self.assertRaisesRegex(ValueError,'INVALID_OBSERVATION'):
            s.accept(Observation('obs','drink',8.,'map',(0.,0.,1.),True),10.)
        s.accept(Observation('obs','drink',10.,'map',(0.,0.,1.),True),10.)
        with self.assertRaisesRegex(ValueError,'GRASP_NOT_CONFIRMED'):
            s.confirm_grasp('drink',{'f':0.},['f'],[.5])
        s.confirm_grasp('drink',{'f':.5},['f'],[.5])
        self.assertEqual(s.held_object,'drink')

    def test_recording(self):
        w=AsyncRecorder(self.path/'run',{'simulated':True})
        for i in range(100): self.assertTrue(w.submit({'n':i}))
        w.close()
        summary=json.loads((self.path/'run/summary.json').read_text())
        self.assertTrue(summary['writer_complete'])
        self.assertEqual(summary['written'],100)

    def test_recorder_bounded(self):
        with patch('bindu_recording.recorder.Thread.start'):
            w=AsyncRecorder(self.path/'bounded',{},capacity=1)
        self.assertTrue(w.submit({'n':1}))
        self.assertFalse(w.submit({'n':2}))
        self.assertEqual(w.queue.qsize(),1)
        self.assertEqual(w.dropped,1)

    def test_recorder_invalid_data_fails(self):
        w=AsyncRecorder(self.path/'invalid',{})
        w.submit({'n':float('nan')})
        w.close()
        self.assertEqual(w.error,'ValueError')
        self.assertFalse(json.loads((self.path/'invalid/summary.json').read_text())['writer_complete'])


    def test_hand_driver_can_be_replaced_alone(self):
        replacement = SimJointDriver(['f'], .1)
        drivers = {**self.drivers, 'hand': replacement}
        io = RobotIO(self.p, drivers, self.base)
        io.write_joints({'f': .5})
        for i in range(1,11): io.read_feedback(10+i*.01, .01)
        self.assertAlmostEqual(io.last_feedback.positions['f'], .01)
        self.assertEqual(self.drivers['arm'].target, {'a': 0., 'b': 0.})
        self.assertEqual(self.base.velocity, (0., 0.))

    def test_route_rejects_mixed_device_write(self):
        with self.assertRaisesRegex(RuntimeError, 'JOINT_ROUTE_MISMATCH'):
            self.io.write_joints({'a': .2, 'f': .2})
        self.assertEqual(self.drivers['arm'].target['a'], 0.)
        self.assertEqual(self.drivers['hand'].target['f'], 0.)

    def test_one_stale_device_cannot_be_hidden(self):
        self.io.read_feedback(10., .01)
        self.drivers['hand'].inject_fault('feedback_loss')
        feedback = self.io.read_feedback(10.5, .01)
        self.assertEqual(feedback.stamp, 10.)
        self.assertEqual(self.io.feedback.source_stamps, {'arm': 10.5, 'hand': 10., 'base': 10.5})

    def test_stop_failure_still_stops_other_devices(self):
        self.io.write_base((.2, 0.))
        self.io.write_joints({'f': .5})
        with patch.object(self.drivers['arm'], 'request_stop', side_effect=RuntimeError('offline')):
            self.e.halt(10., revoke=True)
        self.assertEqual(self.io.stop_failures, ('arm',))
        self.assertEqual(self.base.velocity, (0., 0.))
        self.assertEqual(self.drivers['hand'].target, self.drivers['hand'].positions)
        self.assertFalse(self.e.lease_id)

    def test_bad_feedback_revokes_command(self):
        self.e.submit(self.m, 10.)
        self.drivers['hand'].positions['f'] = float('nan')
        self.drivers['hand'].target['f'] = float('nan')
        self.e.tick(10.01)
        self.assertEqual(self.e.results['cmd'].code, 'DEVICE_FEEDBACK_ERROR')
        self.assertFalse(self.e.lease_id)

if __name__=='__main__': unittest.main()
