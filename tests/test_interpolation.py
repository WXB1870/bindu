"""Whole-curve limits and execution timeline/stop regressions; no ROS required."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import weakref
from dataclasses import replace
from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_execution.interpolation import Curve, State, Timeline, fit_target, fit_stop, fit_online
from bindu_execution.executor import Executor, Rejected
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO


class Interpolation(unittest.TestCase):
    def setUp(self):
        self.p = Profile('test', {'arm':['a']}, {'a':[-1.,1.]}, 1.5,
                         ('joint_position',), 'test')
        self.d = SimJointDriver(['a'], 1.5)
        self.e = Executor(self.p, RobotIO(self.p, {'arm':self.d}, SimBaseDriver()))
        lease, epoch = self.e.acquire('vr', ['arm'], 5., 10.)
        self.m = Motion('first', lease, epoch, 'test', 't', '', 'joint_target',
                        'arm', 10., 1., names=('a',), positions=(.6,))

    def tick_to(self, end):
        while self.e.last_tick < end-1e-8:
            self.e.tick(min(end, self.e.last_tick+.01))
        if self.e.last_tick < end:
            self.e.tick(end)

    def assert_bounded(self, state):
        self.assertLessEqual(abs(state.q[0]), 1.+1e-8)
        self.assertLessEqual(abs(state.v[0]), self.p.speed('a')+1e-8)
        self.assertLessEqual(abs(state.a[0]), self.p.acceleration('a')+1e-8)
        self.assertLessEqual(abs(state.j[0]), self.p.jerk('a')+1e-8)

    def test_one_polynomial_derivatives(self):
        path = fit_target(self.p, ('a',), State((.1,),(.15,),(-.4,)), (.7,), 0.)
        h = 1e-5
        for i in range(1,100):
            t = path.end*i/100
            left, s, right = path.sample(t-h), path.sample(t), path.sample(t+h)
            for lhs, rhs in (((right.q[0]-left.q[0])/(2*h),s.v[0]),
                             ((right.v[0]-left.v[0])/(2*h),s.a[0]),
                             ((right.a[0]-left.a[0])/(2*h),s.j[0])):
                self.assertAlmostEqual(lhs,rhs,places=5)
            self.assert_bounded(s)

    def test_interior_overshoot_is_rejected(self):
        c = Curve.between(0., 1., State((.99,),(.5,),(0.,)), State.rest((.99,)))
        self.assertGreater(c.sample(.2).q[0],1.)
        self.assertFalse(c.valid(self.p, ('a',)))

    def test_update_uses_reference_not_lagging_feedback(self):
        self.d.max_speed = .1
        self.e.submit(self.m,10.)
        self.tick_to(10.2)
        old = self.e.reference
        self.assertGreater(old.q[0],self.e.feedback.positions['a'])
        self.e.submit(replace(self.m,command_id='new',stamp=10.2,positions=(-.2,)),10.2)
        new = self.e.path.sample(10.2)
        for a,b in zip((old.q,old.v,old.a),(new.q,new.v,new.a)):
            self.assertAlmostEqual(a[0],b[0],places=10)

    def test_stream_reversals_and_jitter(self):
        self.e.submit(self.m,10.)
        now = 10.
        samples = [(now,self.e.reference.q[0])]
        for i in range(1,401):
            now += (.006,.014,.009,.011)[i%4]
            self.e.tick(now)
            target = .6*math.sin((now-10.)*3)
            old = self.e.reference
            self.e.submit(replace(self.m,command_id=str(i),stamp=now,positions=(target,)),now)
            samples.append((now,self.e.reference.q[0]))
            if old:
                self.assert_bounded(self.e.reference)
                self.assertAlmostEqual(old.q[0], self.e.reference.q[0], places=9)
                self.assertAlmostEqual(old.v[0], self.e.reference.v[0], places=9)
                self.assertAlmostEqual(old.a[0], self.e.reference.a[0], places=9)

        # Divided differences of published positions independently bound the
        # realized derivatives, including irregular intervals and retargets.
        times=[t for t,q in samples]
        values=[q for t,q in samples]
        for order,limit in enumerate((self.p.speed('a'),self.p.acceleration('a'),self.p.jerk('a')),1):
            values=[(values[i+1]-values[i])/(times[i+order]-times[i]) for i in range(len(values)-1)]
            self.assertLessEqual(max(abs(v)*math.factorial(order) for v in values),limit+1e-5)

    def test_same_goal_does_not_restart_curve(self):
        self.e.submit(self.m,10.)
        end = self.e.path.end
        for i in range(1,80):
            now = 10+i*.01
            self.e.submit(replace(self.m,command_id=str(i),stamp=now),now)
            self.assertEqual(self.e.path.end,end)
        self.assertAlmostEqual(self.e.reference.q[0],.6)

    def test_controlled_stop_and_ownership_fence(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.2)
        before = self.e.reference
        self.e.halt(10.2,revoke=True)
        self.assertIsNotNone(self.e.stopping)
        self.assertFalse(self.e.lease_id)
        self.assertAlmostEqual(self.e.path.sample(10.2).q[0],before.q[0],places=10)
        with self.assertRaisesRegex(Rejected,'BUSY'):
            self.e.acquire('pi',['arm'],2.,10.2)
        while self.e.stopping:
            self.e.tick(self.e.last_tick+.01)
            if self.e.reference: self.assert_bounded(self.e.reference)
        self.assertEqual(self.e.results['first'].state,'CANCELED')
        self.assertFalse(self.e.io.reference_positions)
        lease,epoch = self.e.acquire('pi',['arm'],2.,self.e.last_tick)
        self.e.submit(replace(self.m,command_id='pi',lease_id=lease,epoch=epoch,stamp=self.e.last_tick),self.e.last_tick)
        self.assertAlmostEqual(self.e.reference.q[0],self.e.feedback.positions['a'])

    def test_stop_with_lease_blocks_new_targets(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.2)
        self.e.halt(10.2)
        with self.assertRaisesRegex(Rejected,'STOPPING'):
            self.e.submit(replace(self.m,command_id='late',stamp=10.2),10.2)

    def test_timer_overrun_does_not_jump_to_late_sample(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        self.e.tick(10.5)
        self.assertEqual(self.e.results['first'].code,'EXECUTION_OVERRUN')
        self.assertFalse(self.e.lease_id)
        self.assertFalse(self.e.io.reference_positions)

    def test_expiry_brakes_at_deadline_inside_tick(self):
        self.e.submit(replace(self.m,valid_for=.235),10.)
        self.tick_to(10.23)
        expected=fit_stop(self.p,('a',),self.e.path.sample(10.235),10.235).sample(10.24)
        self.e.tick(10.24)
        self.assertIsNotNone(self.e.stopping)
        for wanted,actual in zip((expected.q,expected.v,expected.a),
                                 (self.e.reference.q,self.e.reference.v,self.e.reference.a)):
            self.assertAlmostEqual(wanted[0],actual[0],places=9)

    def test_overrun_precedes_expired_lease_sampling(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        self.e.tick(16.)
        self.assertEqual(self.e.results['first'].code,'EXECUTION_OVERRUN')
        self.assertIsNone(self.e.path)

    def test_native_reference_failure_revokes_during_cancel(self):
        self.e.profile=replace(self.p,max_transition_duration=.02)
        self.e.submit(self.m,10.)
        self.e.halt(10.05)
        self.assertFalse(self.e.lease_id)
        self.assertEqual(self.e.results['first'].state,'FAILED')
        self.assertIn('REFERENCE_GENERATION_FAILED',self.e.results['first'].code)

    def test_invalid_replacement_is_transactional(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        old,revision = self.e.path,self.e.revision
        bad = replace(self.m,command_id='bad',mode='finite_trajectory',stamp=10.1,
                      positions=(),offsets=(.01,),points=((-1.,),))
        with self.assertRaisesRegex(Rejected,'DYNAMIC_LIMIT'): self.e.submit(bad,10.1)
        self.assertIs(self.e.path,old)
        self.assertEqual(self.e.revision,revision)
        self.assertEqual(self.e.active.command_id,'first')

    def test_future_replacement_preserves_prefix(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        old = self.e.path
        self.e.submit(replace(self.m,command_id='future',stamp=10.14,positions=(-.2,)),10.1)
        for t in (10.11,10.13,10.14):
            self.assertEqual(self.e.path.sample(t).q,old.sample(t).q)
        self.tick_to(10.15)
        self.assert_bounded(self.e.reference)

    def test_native_future_preview_matches_actual_handoff_derivatives(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        old=self.e.path
        expected=old.sample(10.14)
        self.e.submit(replace(self.m,command_id='future_native',stamp=10.14,positions=(-.2,)),10.1)
        self.tick_to(10.14)
        for wanted,actual in zip((expected.q,expected.v,expected.a),
                                 (self.e.reference.q,self.e.reference.v,self.e.reference.a)):
            self.assertAlmostEqual(wanted[0],actual[0],places=9)
        self.tick_to(10.15)
        self.assert_bounded(self.e.reference)

    def test_continuous_future_stream_releases_expired_paths(self):
        self.e.submit(self.m,10.)
        retired = []
        for i in range(1,2001):
            now = 10+i*.01
            if i%50 == 0:
                self.e.renew(self.m.lease_id,self.m.epoch,now)
            old = self.e.path
            times = (now,now+.015,now+.04)
            expected = [old.sample(t) for t in times]
            self.e.submit(replace(self.m,command_id=str(i),stamp=now+.04,
                                  positions=(.1*math.sin(i*.005),)),now)
            for t,wanted in zip(times,expected):
                actual = self.e.path.sample(t)
                for lhs,rhs in zip((wanted.q,wanted.v,wanted.a),(actual.q,actual.v,actual.a)):
                    self.assertAlmostEqual(lhs[0],rhs[0],places=9)
            self.assert_bounded(self.e.reference)
            node,depth = self.e.path,0
            while isinstance(node,Timeline):
                depth += 1
                node = node.before
            self.assertLessEqual(depth,5)  # Only the next 40ms, with rounding margin.
            retired.append(weakref.ref(node))
        del old,node
        self.assertTrue(all(ref() is None for ref in retired[:-10]))
        self.tick_to(30.05)
        self.assertNotIsInstance(self.e.path,Timeline)

    def test_same_future_switch_replaces_unreachable_suffix(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.1)
        old = self.e.path
        before = old.sample(10.12)
        at_switch = old.sample(10.14)
        retired = []
        for i in range(1200):
            self.e.submit(replace(self.m,command_id=str(i),stamp=10.14,
                                  positions=((.1,-.1)[i%2],)),10.1)
            retired.append(weakref.ref(self.e.path.after))
        self.assertIs(self.e.path.before,old)
        self.assertEqual(self.e.path.sample(10.12),before)
        actual = self.e.path.sample(10.14)
        for lhs,rhs in zip((at_switch.q,at_switch.v,at_switch.a),(actual.q,actual.v,actual.a)):
            self.assertAlmostEqual(lhs[0],rhs[0],places=9)
        self.assertTrue(all(ref() is None for ref in retired[:-1]))

    def test_future_pruning_preserves_timed_path_and_saved_snapshot(self):
        self.e.submit(self.m,10.)
        for i in range(1,5):
            self.e.submit(replace(self.m,command_id=str(i),stamp=10.1+i*.01,
                                  positions=(.1*i,)),10.1)
        self.e.submit(replace(self.m,command_id='timed',mode='finite_trajectory',
                              stamp=10.15,positions=(),offsets=(1.,),points=((.2,),),
                              valid_for=2.),10.1)
        snapshot = self.e.path
        times = (10.1,10.115,10.12,10.139,10.15,10.4,11.15)
        expected = [snapshot.sample(t) for t in times]
        self.e.tick(10.12)
        self.assertEqual(self.e.path.end,11.15)
        for t,wanted in zip(times,expected):
            self.assertEqual(snapshot.sample(t),wanted)
            if t >= 10.12:
                self.assertEqual(self.e.path.sample(t),wanted)

    def test_future_pruning_keeps_lease_expiry_braking_boundary(self):
        self.e.submit(self.m,10.)
        for i in range(1,20):
            now = 10+i*.01
            self.e.submit(replace(self.m,command_id=str(i),stamp=now+.04,
                                  positions=(.1*math.sin(i*.05),)),now)
        self.e.expires = 10.195
        expected = fit_stop(self.p,('a',),self.e.path.sample(10.195),10.195).sample(10.2)
        self.e.tick(10.2)
        self.assertIsNotNone(self.e.stopping)
        self.assertFalse(self.e.lease_id)
        for lhs,rhs in zip((expected.q,expected.v,expected.a),
                           (self.e.reference.q,self.e.reference.v,self.e.reference.a)):
            self.assertAlmostEqual(lhs[0],rhs[0],places=9)

    def test_native_binding_validates_shapes_and_finite_values(self):
        from bindu_execution import _native
        from importlib.machinery import EXTENSION_SUFFIXES
        self.assertTrue(any(_native.__file__.endswith(s) for s in EXTENSION_SUFFIXES))
        for s in (State((0.,),(),(0.,)),State((float('nan'),),(0.,),(0.,))):
            with self.assertRaises(ValueError): Curve.between(0.,1.,s,State.rest((0.,)))
            with self.assertRaises(ValueError): fit_online(self.p,('a',),s,(.1,),0.)
        for t in (float('nan'),float('inf'),-.1):
            with self.assertRaises(ValueError): Curve.between(0.,t,State.rest((0.,)),State.rest((.1,)))
        with self.assertRaises(ValueError): _native.curve_sample(None,0.)
        with self.assertRaises(ValueError): fit_online(self.p,('a',),State.rest((0.,)),(.1,.2),0.)

    def test_timed_derivatives_and_original_end_are_preserved(self):
        m = replace(self.m,mode='finite_trajectory',positions=(),offsets=(.5,1.),
                    points=((.15,),(.3,)),velocities=((.4,),(0.,)),
                    accelerations=((0.,),(0.,)),valid_for=2.)
        self.e.submit(m,10.)
        self.assertEqual(self.e.path.end,11.)
        self.assertAlmostEqual(self.e.path.sample(10.5).v[0],.4)
        self.assertAlmostEqual(self.e.path.sample(10.5-1e-7).v[0],.4,places=5)

    def test_finite_nonrest_endpoint_and_partial_derivatives_rejected(self):
        m=replace(self.m,mode='finite_trajectory',positions=(),offsets=(.5,),points=((.1,),),velocities=((.1,),))
        with self.assertRaisesRegex(Rejected,'NOT_AT_REST'): self.e.submit(m,10.)
        with self.assertRaisesRegex(Rejected,'DERIVATIVE_LAYOUT'):
            self.e.submit(replace(m,velocities=((),)),10.)

    def test_chunk_trims_expired_prefix_without_retiming(self):
        m=replace(self.m,mode='joint_reference_segment',positions=(),stamp=9.8,
            offsets=(0.,.1,.7,1.2),points=((-1.,),(-1.,),(.1,),(.2,)),
            expected_revision=self.e.revision,valid_for=2.)
        self.e.submit(m,10.)
        self.assertAlmostEqual(self.e.path.sample(10.5).q[0],.1)
        self.assertAlmostEqual(self.e.path.sample(11.).q[0],.2)
        self.assertAlmostEqual(self.e.path.curves[0].end,10.5)
        self.tick_to(11.1)
        self.assertEqual(self.e.results['first'].code,'BUFFER_EXHAUSTED')
        self.assertIsNone(self.e.active)

    def test_chunk_stale_revision_and_no_future_knots(self):
        m=replace(self.m,mode='joint_reference_segment',positions=(),offsets=(.5,),points=((.1,),))
        with self.assertRaisesRegex(Rejected,'STALE_REVISION'): self.e.submit(m,10.)
        with self.assertRaisesRegex(Rejected,'EXPIRED_TRAJECTORY_PREFIX'):
            self.e.submit(replace(m,stamp=9.,valid_for=2.,expected_revision=self.e.revision),10.)

    def test_chunk_nonzero_endpoint_has_certified_stop_tail(self):
        m=replace(self.m,mode='joint_reference_segment',positions=(),offsets=(.5,),points=((.1,),),
            velocities=((.3,),),expected_revision=self.e.revision)
        self.e.submit(m,10.)
        self.assertGreater(self.e.path.end,10.5)
        self.assertAlmostEqual(self.e.path.sample(10.5).v[0],.3)
        for curve in self.e.path.curves: self.assertTrue(curve.valid(self.p,('a',)))
        self.tick_to(10.9)
        self.assertIsNone(self.e.stopping)
        self.assertEqual(self.e.results['first'].code,'BUFFER_EXHAUSTED')

    def test_braking_envelopes_near_both_limits(self):
        for goal in (-1.,-.9,0.,.9,1.):
            path=fit_target(self.p,('a',),State.rest((0.,)),(goal,),0.)
            for i in range(1,100):
                t=path.end*i/100
                stop=fit_stop(self.p,('a',),path.sample(t),t)
                self.assertTrue(all(c.valid(self.p,('a',)) for c in stop.curves))
                final=stop.sample(stop.end)
                self.assertAlmostEqual(final.v[0],0.,places=8)
                self.assertAlmostEqual(final.a[0],0.,places=8)

    def test_large_online_move_has_separate_transition_horizon(self):
        p=replace(self.p,max_speed=.1)
        path=fit_target(p,('a',),State.rest((0.,)),(1.,),0.)
        self.assertGreater(path.end,p.stop_timeout)
        self.assertTrue(all(c.valid(p,('a',)) for c in path.curves))

    def test_stop_waits_for_feedback_or_reports_timeout(self):
        self.e.submit(self.m,10.)
        self.tick_to(10.2)
        self.d.max_speed=0.
        self.e.halt(10.2,revoke=True)
        self.tick_to(11.)
        self.assertTrue(self.e.stopping)
        self.tick_to(15.3)
        self.assertEqual(self.e.results['first'].code,'STOP_FEEDBACK_TIMEOUT')
        self.assertFalse(self.e.lease_id)

    def test_finite_next_command_uses_held_reference(self):
        m=replace(self.m,mode='finite_trajectory',positions=(),offsets=(.5,),points=((.3,),))
        self.e.submit(m,10.)
        self.tick_to(10.5)
        self.assertEqual(self.e.results['first'].state,'SUCCEEDED')
        self.d.positions['a']=.299
        self.d.target['a']=.299
        self.e.submit(replace(self.m,command_id='next',stamp=10.5),10.5)
        self.assertAlmostEqual(self.e.reference.q[0],.3)

    def test_release_after_finite_tolerance_waits_for_resting_reference(self):
        m=replace(self.m,mode='finite_trajectory',positions=(),offsets=(.5,),points=((.3,),))
        self.e.submit(m,10.)
        self.tick_to(10.5)
        self.d.positions['a']=.29
        self.e.tick(10.5)
        self.e.halt(10.5,revoke=True)
        self.assertTrue(self.e.stopping)
        self.assertIsNone(self.e.path)
        self.assertAlmostEqual(self.d.target['a'],.3)
        with self.assertRaisesRegex(Rejected,'BUSY'):
            self.e.acquire('new',['arm'],2.,10.5)
        self.tick_to(10.6)
        self.assertIsNone(self.e.stopping)
        self.assertAlmostEqual(self.d.positions['a'],.3)

    def test_per_joint_configuration_validation(self):
        p=replace(self.p,joint_dynamics={'a':{'velocity':.2,'acceleration':.8,'jerk':4.}})
        curve=fit_target(p,('a',),State.rest((0.,)),(.2,),0.).curves[0]
        self.assertTrue(curve.valid(p,('a',)))
        self.assertGreater(curve.duration,1.)
        root=Path(__file__).resolve().parents[1]
        raw=json.loads((root/'src/integration/bindu_runtime/config/wheel_sim.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'bad.json'
            for cfg in ({'max_jerk':float('nan')},{'control_period':1.,'max_tick_gap':.1},
                        {'joint_dynamics':{'unknown':{'velocity':1.}}}):
                raw['execution']=cfg;path.write_text(json.dumps(raw))
                with self.assertRaises(ValueError): Profile.load(path)

    def test_three_axis_timed_p2p_respects_individual_limits(self):
        names=('a','b','c')
        p=replace(self.p,groups={'arm':list(names)},limits={n:[-1.,1.] for n in names},
                  joint_dynamics={n:{'velocity':v} for n,v in zip(names,(.2,.6,1.2))})
        driver=SimJointDriver(names,1.5)
        e=Executor(p,RobotIO(p,{'arm':driver},SimBaseDriver()))
        lease,epoch=e.acquire('planner',['arm'],5.,10.)
        goal=(.2,-.3,.4)
        m=replace(self.m,lease_id=lease,epoch=epoch,names=names,mode='finite_trajectory',
                  positions=(),offsets=(2.,),points=(goal,),valid_for=3.)
        e.submit(m,10.)
        self.assertEqual(e.path.end,12.)
        for i in range(1,200):
            e.tick(10.+i*.01)
            self.assertIsNotNone(e.active)
            s=e.reference
            for axis,n in enumerate(names):
                self.assertLessEqual(abs(s.q[axis]),1.+1e-8)
                self.assertLessEqual(abs(s.v[axis]),p.speed(n)+1e-8)
                self.assertLessEqual(abs(s.a[axis]),p.acceleration(n)+1e-8)
                self.assertLessEqual(abs(s.j[axis]),p.jerk(n)+1e-8)
        e.tick(12.)
        self.assertEqual(e.results[m.command_id].state,'SUCCEEDED')
        self.assertEqual(e.results[m.command_id].code,'FEEDBACK_CONFIRMED')
        for n,q in zip(names,goal): self.assertAlmostEqual(e.io.reference_positions[n],q)

    def test_stream_timed_chunk_stream_transitions(self):
        self.e.submit(self.m,10.)
        replacements=(
            replace(self.m,command_id='timed',stamp=10.1,mode='finite_trajectory',
                    positions=(),offsets=(.5,1.),points=((.2,),(.3,)),
                    velocities=((.2,),(0.,)),valid_for=2.),
            replace(self.m,command_id='chunk',stamp=10.2,mode='joint_reference_segment',
                    positions=(),offsets=(.5,1.),points=((.25,),(.35,)),
                    velocities=((.15,),(.15,)),valid_for=2.),
            replace(self.m,command_id='stream',stamp=10.3,positions=(-.1,),valid_for=2.))
        for m in replacements:
            self.tick_to(m.stamp)
            before=self.e.reference
            if m.mode=='joint_reference_segment': m=replace(m,expected_revision=self.e.revision)
            self.e.submit(m,m.stamp)
            after=self.e.reference
            for a,b in zip((before.q,before.v,before.a),(after.q,after.v,after.a)):
                self.assertAlmostEqual(a[0],b[0],places=9)
            self.assertIsNone(self.e.stopping)
            self.assertEqual(self.e.results[m.command_id].state,'ACCEPTED')
        while self.e.last_tick<11.8-1e-8:
            self.e.tick(min(11.8,self.e.last_tick+.01))
            self.assert_bounded(self.e.reference)
        self.assertAlmostEqual(self.e.reference.q[0],-.1)
        self.e.halt(11.8)
        while self.e.stopping: self.e.tick(self.e.last_tick+.01)
        self.assertEqual(self.e.results['stream'].state,'CANCELED')


if __name__=='__main__': unittest.main()
