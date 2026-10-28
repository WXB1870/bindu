"""Same position semantics on PhysX: measured state, post-step ACK and one-shot hold."""
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest

from bindu_contracts.profile import Profile
from bindu_hardware.drivers.physics import PhysicsPositionState

class PhysicsRealtimeTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('snowbot_isaac',ROOT/'tools/simulation/snowbot_isaac.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.module=module
        self.monitor=module.RealtimeStatus(10.,.15)

    def test_explicit_unload_after_interruption_removes_the_scene_tool(self):
        request=self.module.scene_request
        self.assertIsNone(request(dict(load_state='empty',load_source='configured_initial_state'),True))
        self.assertIsNone(request(dict(load_state='unknown',load_source='interrupted_loading'),True))
        self.assertEqual(request(dict(load_state='empty',load_source='operator_load_confirmation'),True),'remove')
        self.assertIsNone(request(dict(load_state='empty',load_source='operator_load_confirmation'),False))
        self.assertEqual(request(dict(load_state='held',load_source='operator_load_confirmation'),False),'install')

    def test_step_stall_invalidates_the_same_sample_and_latches_with_commands(self):
        self.assertFalse(self.monitor.update(11.,1.,True))
        self.assertTrue(self.monitor.ready)
        self.assertTrue(self.monitor.update(11.34,1.004,True))
        self.assertFalse(self.monitor.ready)
        self.assertEqual(self.monitor.fault,'PHYSICS_REALTIME_LAG')
        self.assertFalse(self.monitor.update(11.35,1.35,True))
        self.assertFalse(self.monitor.ready)

    def test_uncommanded_warmup_recovers_without_faking_ready(self):
        self.assertFalse(self.monitor.update(11.34,1.004,False))
        self.assertFalse(self.monitor.ready)
        self.assertEqual(self.monitor.fault,'')
        self.monitor.update(11.35,1.35,False)
        self.assertTrue(self.monitor.ready)
        self.assertAlmostEqual(self.monitor.maximum_lag,.336)

    def test_native_velocity_limits_use_profile_intersection_and_verify_readback(self):
        import numpy as np
        limits=np.array([[4.19,.5]])
        def get():return limits.copy()
        def set_limits(values):limits[:]=values
        body=S(dof_names=['arm','waist'],_articulation_view=S(
            get_joint_max_velocities=get,set_max_joint_velocities=set_limits))
        result=self.module.apply_velocity_limits(body,S(speed=lambda n:{'arm':1.5,'waist':1.2}[n]))
        self.assertEqual(result,{'arm':1.5,'waist':.5})
        limits[:]=float('nan')
        with self.assertRaisesRegex(ValueError,'PHYSICS_NATIVE_VELOCITY_LIMITS'):
            self.module.apply_velocity_limits(body,S())


class PositionPhysicsTests(unittest.TestCase):
    def setUp(self):
        # Shared-progress semantics use an explicit uniform fixture; field
        # waist/arm tuning is verified separately by the target-wire tests.
        self.profile=replace(Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json'),
                             max_speed=1.5,joint_dynamics={},digest='uniform_physics_fixture')
        self.q=dict.fromkeys(self.profile.limits,0.)
        self.core=PhysicsPositionState(self.profile,'boot',self.q)

    def command(self,seq=1,operation='set',q=.1,group='right_tool_axis',**changes):
        vector=lambda:S(x=0.,y=0.,z=0.)
        value=dict(stamp=S(sec=10,nanosec=0),simulator_id='boot',sender_id='executor',sequence=seq,
            profile_hash=self.profile.digest,valid_for=.2,operation=operation,resource_group=group,
            joint_names=[] if operation=='hold' else self.profile.groups[group],
            positions=[] if operation=='hold' else [q]*len(self.profile.groups[group]),
            velocity=S(linear=vector(),angular=vector()))
        value.update(changes);return S(**value)

    def test_position_is_not_feedback_and_ack_requires_physics_step(self):
        self.core.accept(self.command(),10.01,self.q)
        self.assertEqual(self.core.targets['arm_r_joint_8'],.1)
        self.assertEqual(self.q['arm_r_joint_8'],0.)
        self.assertEqual(self.core.applied_sequences,{})
        self.core.stepped()
        self.assertEqual(self.core.applied_sequences,{'right_tool_axis':1})

    def test_reference_uses_shared_progress_and_matching_velocity_without_faking_measurement(self):
        self.core.targets.update(arm_l_joint_1=.3,arm_r_joint_1=.15,body_joint_1=.1)
        positions,velocity=self.core.reference_step(.01)
        self.assertAlmostEqual(positions['arm_l_joint_1'],.015)
        self.assertAlmostEqual(positions['arm_r_joint_1'],.0075)
        self.assertAlmostEqual(positions['body_joint_1'],.005)
        for n in positions:
            self.assertAlmostEqual(velocity[n],positions[n]/.01)
            self.assertLessEqual(abs(velocity[n]),self.profile.speed(n)+1e-12)
        self.assertEqual(self.q['arm_l_joint_1'],0.)
        self.assertEqual(self.core.applied_sequences,{})
        for bad in (0.,-.01,.101,float('nan')):
            with self.assertRaisesRegex(ValueError,'PHYSICS_REFERENCE_TIMESTEP'):
                self.core.reference_step(bad)

    def test_hold_discards_pending_reference_lead_and_zeroes_its_velocity(self):
        self.core.targets['arm_r_joint_1']=.3;self.core.reference_step(.01)
        self.q['arm_r_joint_1']=.003
        self.core.accept(self.command(operation='hold',group='right_arm'),10.01,self.q)
        self.q['arm_r_joint_1']=.004
        self.core.expire(10.3,self.q)
        positions,velocity=self.core.reference_step(.004)
        self.assertEqual(positions['arm_r_joint_1'],.003)
        self.assertEqual(velocity['arm_r_joint_1'],0.)

    def test_grippers_and_head_do_not_throttle_the_coupled_arm_progress(self):
        self.core.targets.update(arm_l_joint_1=.3,arm_r_joint_1=.15,body_joint_1=.1,
                                 arm_l_joint_8=.8,head_joint_1=.5)
        positions,velocity=self.core.reference_step(.01)
        self.assertAlmostEqual(positions['arm_l_joint_1'],.015)
        self.assertAlmostEqual(positions['arm_r_joint_1'],.0075)
        for n in ('arm_l_joint_8','head_joint_1'):
            self.assertAlmostEqual(velocity[n],self.profile.speed(n))
        self.assertEqual(self.q['arm_l_joint_8'],0.)

    def test_hold_captures_measured_position_once_and_never_closes(self):
        self.core.accept(self.command(),10.01,self.q)
        self.q['arm_r_joint_8']=.018
        self.core.accept(self.command(2,'hold'),10.02,self.q)
        self.q['arm_r_joint_8']=.019
        self.core.accept(self.command(3,'hold'),10.03,self.q)
        self.core.expire(10.3,self.q)
        self.assertEqual(self.core.targets['arm_r_joint_8'],.018)
        self.assertEqual(self.core.hold_positions['arm_r_joint_8'],.018)

    def test_first_hold_captures_measured_state_even_without_prior_motion(self):
        self.q['arm_r_joint_8']=.002
        self.core.accept(self.command(operation='hold'),10.01,self.q)
        self.assertEqual(self.core.hold_positions['arm_r_joint_8'],.002)

    def test_stop_ack_waits_for_measured_stable_rest_and_keeps_drive_target(self):
        self.core.accept(self.command(operation='hold',group='right_arm'),10.01,self.q)
        velocity=dict.fromkeys(self.q,0.)
        # Recorded shoulder bias: the force drive rests 2.126 mrad away from
        # its requested hold, outside the unchanged 2 mrad arrival tolerance.
        self.q['arm_r_joint_1']=-.002126
        self.core.observe_holds(self.q,velocity,10.02)
        self.assertFalse(self.core.hold_acknowledgments)  # No post-step ACK yet.
        self.core.stepped()
        velocity['arm_r_joint_1']=.02
        self.core.observe_holds(self.q,velocity,10.03)
        self.assertFalse(self.core.hold_acknowledgments)
        velocity['arm_r_joint_1']=0.
        self.core.observe_holds(self.q,velocity,10.04)
        self.core.observe_holds(self.q,velocity,10.09)
        self.assertFalse(self.core.hold_acknowledgments)
        self.core.observe_holds(self.q,velocity,10.15)
        ack=self.core.hold_acknowledgments['right_arm']
        self.assertEqual(ack['sequence'],1)
        self.assertEqual(ack['stamp'],10.15)
        self.assertEqual(ack['positions']['arm_r_joint_1'],-.002126)
        self.assertEqual(self.core.hold_positions['arm_r_joint_1'],0.)
        self.assertEqual(self.core.targets['arm_r_joint_1'],0.)
        self.q['arm_r_joint_1']=-.02
        self.core.observe_holds(self.q,velocity,10.3)
        self.assertEqual(ack['positions']['arm_r_joint_1'],-.002126)  # Never chase drift.

    def test_timeout_hold_cannot_ack_an_explicit_stop_and_new_set_clears_ack(self):
        velocity=dict.fromkeys(self.q,0.)
        self.core.accept(self.command(),10.01,self.q)
        self.core.stepped();self.core.expire(10.3,self.q)
        for at in (10.31,10.5):self.core.observe_holds(self.q,velocity,at)
        self.assertFalse(self.core.hold_acknowledgments)
        self.core.accept(self.command(2,'hold',stamp=S(sec=11,nanosec=0)),11.01,self.q)
        self.core.stepped()
        for at in (11.02,11.2):self.core.observe_holds(self.q,velocity,at)
        self.assertIn('right_tool_axis',self.core.hold_acknowledgments)
        self.core.accept(self.command(3,stamp=S(sec=11,nanosec=200000000)),11.21,self.q)
        self.assertFalse(self.core.hold_acknowledgments)

    def test_measured_boundary_bias_can_ack_without_clipping_actual_or_drive_target(self):
        self.q['arm_r_joint_3']=-.5799692869
        self.core.accept(self.command(operation='hold',group='right_arm'),10.01,self.q)
        self.core.stepped();self.q['arm_r_joint_3']=-.5800696015
        for at in (10.02,10.13):self.core.observe_holds(self.q,dict.fromkeys(self.q,0.),at)
        ack=self.core.hold_acknowledgments['right_arm']
        self.assertEqual(ack['positions']['arm_r_joint_3'],self.q['arm_r_joint_3'])
        self.assertEqual(self.core.hold_positions['arm_r_joint_3'],-.5799692869)

    def test_measured_overrun_beyond_feedback_tolerance_cannot_ack(self):
        self.core.accept(self.command(operation='hold',group='right_arm'),10.01,self.q)
        self.core.stepped();self.q['arm_r_joint_3']=-.582001
        for at in (10.02,10.13):self.core.observe_holds(self.q,dict.fromkeys(self.q,0.),at)
        self.assertFalse(self.core.hold_acknowledgments)

    def test_tiny_out_of_limit_motion_target_is_still_rejected(self):
        values=[0.]*7;values[2]=-.5800001
        with self.assertRaisesRegex(ValueError,'SIM_JOINT_LIMIT'):
            self.core.accept(self.command(group='right_arm',positions=values),10.01,self.q)

    def test_missing_samples_cannot_supply_a_stable_hold_window(self):
        velocity=dict.fromkeys(self.q,0.)
        self.core.accept(self.command(operation='hold'),10.01,self.q);self.core.stepped()
        self.core.observe_holds(self.q,velocity,10.02)
        self.core.observe_holds(self.q,velocity,10.5)
        self.assertFalse(self.core.hold_acknowledgments)
        self.core.observe_holds(self.q,velocity,10.61)
        self.assertIn('right_tool_axis',self.core.hold_acknowledgments)

    def test_hold_settling_resets_on_motion_drift_or_invalid_feedback(self):
        velocity=dict.fromkeys(self.q,0.)
        self.core.accept(self.command(operation='hold'),10.01,self.q);self.core.stepped()
        self.core.observe_holds(self.q,velocity,10.02)
        velocity['arm_r_joint_8']=.02
        self.core.observe_holds(self.q,velocity,10.10)
        velocity['arm_r_joint_8']=0.
        self.core.observe_holds(self.q,velocity,10.11)
        self.q['arm_r_joint_8']=.003
        self.core.observe_holds(self.q,velocity,10.19)
        self.core.observe_holds(self.q,velocity,10.22)
        self.assertFalse(self.core.hold_acknowledgments)
        velocity['arm_r_joint_8']=float('nan')
        self.core.observe_holds(self.q,velocity,10.23)
        velocity['arm_r_joint_8']=0.
        for at in (10.24,10.35):self.core.observe_holds(self.q,velocity,at)
        self.assertIn('right_tool_axis',self.core.hold_acknowledgments)

    def test_timeout_holds_only_commanded_group_at_measured_position(self):
        self.core.accept(self.command(),10.01,self.q)
        self.q['arm_r_joint_8']=.025
        self.assertEqual(self.core.expire(10.3,self.q),['right_tool_axis'])
        self.assertEqual(self.core.targets['arm_r_joint_8'],.025)
        self.assertEqual(self.core.targets['arm_l_joint_8'],0.)

    def test_stale_replay_wrong_boot_and_executor_never_change_target(self):
        self.core.accept(self.command(),10.01,self.q)
        for c,now in [(self.command(),10.02),(self.command(2,simulator_id='other'),10.02),
                      (self.command(2,sender_id='other'),10.02),(self.command(2),10.3)]:
            with self.assertRaises(ValueError):self.core.accept(c,now,self.q)
            self.assertEqual(self.core.targets['arm_r_joint_8'],.1)

    def test_nonfinite_and_limit_violations_are_rejected(self):
        for value in [float('nan'),2.]:
            with self.assertRaises(ValueError):self.core.accept(self.command(q=value),10.01,self.q)

    def test_live_config_uses_contact_and_no_artificial_delay(self):
        c=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_shovel_physics.json').read_text())
        self.assertEqual(c['grasp_model'],'contact')
        self.assertLessEqual(c['live']['physics_dt_s'],.005)
        self.assertFalse(any('delay' in key for key in c['live']))

    def test_live_config_rejects_nonfinite_or_unsafe_clock_parameters_without_isaac(self):
        import importlib.util,copy
        spec=importlib.util.spec_from_file_location('snowbot_isaac',ROOT/'tools/simulation/snowbot_isaac.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        c=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_shovel_physics.json').read_text())
        module.validate_live_config(c)
        for key,value in [('physics_dt_s',float('nan')),('max_wall_lag_s',1.),('command_timeout_s',.1),
                          ('stop_settle_s',0.),('stop_settle_s',float('nan')),('headless','true')]:
            changed=copy.deepcopy(c);changed['live'][key]=value
            with self.assertRaises(ValueError):module.validate_live_config(changed)


class PhysicsPositionStopTests(unittest.TestCase):
    def test_external_simulation_factory_loads_shared_physics_io(self):
        from unittest.mock import Mock, patch
        from bindu_runtime.simulation_devices import PhysicsPositionIO
        from snowbot_hardware.position_sim import position_devices
        node=S(device_backend='external_simulation', now=lambda:10.,
               declare_parameter=Mock(), get_parameter=lambda name:S(value=True))
        devices={group:S(transport=self.wire) for group in ('left_arm','right_arm','waist')}
        with patch('snowbot_hardware.position_sim.assemble_devices',return_value=(S(),devices)):
            io, actual=position_devices(node,self.profile)
        self.assertIsInstance(io,PhysicsPositionIO)
        self.assertIs(io.physics,self.wire)
        self.assertIs(actual,devices)

    def setUp(self):
        from bindu_runtime.simulation_devices import PhysicsPositionIO
        from bindu_execution.delegated_position import DelegatedPositionExecutor
        self.now=10.;self.sequence=0;self.sent=[]
        self.profile=Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        self.q=dict.fromkeys(self.profile.limits,0.);self.v=dict.fromkeys(self.q,0.)
        self.wire=S(health={},simulator_id='boot',failed=False,send=self.send)
        self.io=PhysicsPositionIO(self.profile,lambda g,q:None,lambda:self.now,
                                 command_enabled=True,simulated=True,physics=self.wire)
        self.observe()
        self.engine=DelegatedPositionExecutor(self.profile,self.io,steady=lambda:self.now)
        groups=('waist','left_arm','right_arm');self.engine.acquire('test',groups,2.,self.now)
        self.io.write_joints({n:min(self.profile.limits[n][1],max(self.profile.limits[n][0],.01))
                              for g in groups for n in self.profile.groups[g]})

    def send(self,group,operation):
        self.sequence+=1;self.sent.append((group,operation,self.sequence))
        return self.sequence

    def observe(self):
        self.io.observe(self.q,self.v,self.now,self.now)

    def tick(self,dt=.02):
        self.now+=dt;self.observe();self.engine.tick(self.now)

    def acknowledge(self,**changes):
        h=dict(stamp=self.now,ready=True,simulator_id='boot',
               applied_sequences=dict(self.io.hold_sequences),
               hold_positions=dict(self.io.stop_targets),
               hold_acknowledgments={g:dict(sequence=seq,stamp=self.now,
                    positions={n:self.q[n] for n in self.profile.groups[g]})
                    for g,seq in self.io.hold_sequences.items()})
        h.update(changes);self.wire.health=h

    def test_hold_uses_device_position_after_all_post_step_acks(self):
        self.engine.halt(self.now)
        self.assertEqual({operation for _,operation,_ in self.sent},{'hold'})
        self.assertEqual(len(self.sent),3)
        # Reproduce the observed 7.7mrad difference, beyond the unchanged 2mrad
        # stop tolerance. A newer physical hold point must be acknowledged.
        self.q['arm_r_joint_1']=.0077;self.tick()
        self.assertIsNotNone(self.engine.stopping)
        self.acknowledge();self.v['arm_r_joint_1']=.2;self.tick()
        self.assertIsNotNone(self.engine.stopping)
        self.v['arm_r_joint_1']=0.;self.tick()
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)
        self.assertEqual(self.profile.feedback_tolerances['joint_position'],.002)

    def test_hold_ack_waits_for_feedback_at_or_after_ack_sample(self):
        self.engine.halt(self.now);self.now+=.02
        self.q['arm_r_joint_1']=.0077;self.acknowledge()
        self.assertIsNone(self.io.confirmed_stop_targets())
        self.observe()
        self.assertEqual(self.io.confirmed_stop_targets()['arm_r_joint_1'],.0077)

    def test_measured_boundary_bias_releases_stop_with_actual_ack(self):
        self.q['arm_r_joint_3']=-.5800696015;self.now+=.01;self.observe()
        self.engine.halt(self.now);self.now+=.02;self.observe();self.acknowledge()
        self.assertEqual(self.io.confirmed_stop_targets()['arm_r_joint_3'],self.q['arm_r_joint_3'])
        self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping);self.assertFalse(self.engine.stop_fault)

    def test_ack_beyond_measured_boundary_envelope_cannot_release_stop(self):
        self.engine.halt(self.now);self.now+=.02
        self.q['arm_r_joint_3']=-.582001;self.observe();self.acknowledge()
        self.assertIsNone(self.io.confirmed_stop_targets())
        self.engine.tick(self.now)
        self.assertIsNotNone(self.engine.stopping)

    def test_missing_wrong_or_stale_ack_cannot_confirm_even_stationary_joints(self):
        self.engine.halt(self.now);self.tick()
        for changes in ({'stamp':10.}, {'stamp':9.}, {'stamp':self.now+.1},
                        {'ready':False}, {'simulator_id':'old'},
                        {'applied_sequences':{}}, {'hold_acknowledgments':{}},
                        {'hold_acknowledgments':{'right_arm':dict(sequence=1,stamp=self.now,positions={})}}):
            with self.subTest(changes=changes):
                self.acknowledge(**changes)
                self.assertIsNone(self.io.confirmed_stop_targets())
        self.acknowledge();self.wire.failed=True
        self.assertIsNone(self.io.confirmed_stop_targets())

    def test_post_step_acceptance_without_stable_stop_ack_cannot_confirm(self):
        self.engine.halt(self.now);self.tick();self.acknowledge()
        del self.wire.health['hold_acknowledgments']
        self.assertIsNone(self.io.confirmed_stop_targets())
        self.engine.tick(self.now)
        self.assertIsNotNone(self.engine.stopping)

    def test_stable_ack_rejects_wrong_sequence_time_layout_and_nonfinite_position(self):
        self.engine.halt(self.now);self.tick()
        for change in ('sequence','old_stamp','future_stamp','layout','nonfinite'):
            with self.subTest(change=change):
                self.acknowledge()
                ack=self.wire.health['hold_acknowledgments']['right_arm']
                if change=='sequence':ack['sequence']-=1
                elif change=='old_stamp':ack['stamp']=10.
                elif change=='future_stamp':ack['stamp']=self.now+.1
                elif change=='layout':ack['positions'].pop('arm_r_joint_1')
                else:ack['positions']['arm_r_joint_1']=float('nan')
                self.assertIsNone(self.io.confirmed_stop_targets())

    def test_recorded_static_bias_confirms_actual_rest_without_relaxing_tolerance(self):
        self.engine.halt(self.now)
        self.q['arm_r_joint_1']=-.002126;self.tick()
        self.acknowledge()
        self.assertEqual(self.wire.health['hold_positions']['arm_r_joint_1'],0.)
        self.tick()
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)
        self.assertEqual(self.profile.feedback_tolerances['joint_position'],.002)

    def test_ack_position_never_chases_measured_drift(self):
        self.engine.halt(self.now);self.q['arm_r_joint_1']=.0077;self.tick();self.acknowledge()
        self.assertEqual(self.io.confirmed_stop_targets()['arm_r_joint_1'],.0077)
        self.q['arm_r_joint_1']=.02;self.tick()
        self.assertIsNotNone(self.engine.stopping)
        self.acknowledge()  # A changed device hold cannot rewrite the first ACK.
        self.assertIsNone(self.io.confirmed_stop_targets())

    def test_no_ack_or_late_ack_preserves_timeout_and_repeated_stop_footprint(self):
        self.engine.halt(self.now);deadline=self.engine.stop_deadline
        self.tick();self.engine.halt(self.now,emergency=True)
        self.assertEqual(self.engine.stop_deadline,deadline)
        self.assertEqual(len(self.sent),3)
        self.now=deadline+.01;self.observe();self.acknowledge();self.engine.tick(self.now)
        self.assertTrue(self.engine.stop_fault)
        self.assertIn('STOP_UNCONFIRMED',self.engine.stop_fault)

    def test_one_failed_hold_still_attempts_other_groups_and_fails_closed(self):
        send=self.wire.send
        def partial(group,operation):
            if group=='left_arm':raise RuntimeError('delivery failed')
            return send(group,operation)
        self.wire.send=partial
        self.engine.halt(self.now)
        self.assertEqual({g for g,_,_ in self.sent},{'right_arm','waist'})
        self.assertEqual(self.io.stop_failures,('left_arm',))
        self.assertTrue(self.engine.stop_fault)

    def test_idle_stop_never_sends_a_device_command(self):
        self.io.targets.clear();self.engine.halt(self.now)
        self.assertFalse(self.sent)


class PhysicsGripperAcknowledgmentTests(unittest.TestCase):
    def test_old_physics_sample_never_becomes_initial_device_feedback(self):
        from bindu_runtime.simulation_devices import SimulationDevices
        from bindu_interfaces.msg import SimulationFeedback
        from builtin_interfaces.msg import Time
        transport=SimulationDevices.__new__(SimulationDevices)
        transport.node=S(now=lambda:10.05);transport.profile=S(digest='profile')
        transport.simulator_id='';transport.latest=None;transport.failed=False;transport.names=['j']
        msg=SimulationFeedback(stamp=Time(sec=9),simulator_id='boot',profile_hash='profile',sequence=1,ready=True)
        msg.joints.name=['j'];msg.joints.position=[0.];msg.joints.velocity=[0.]
        msg.base_pose.orientation.w=1.
        transport.receive(msg)
        self.assertIsNone(transport.latest)
        msg.stamp=Time(sec=10)
        transport.receive(msg)
        self.assertIs(transport.latest,msg)

    def test_contact_velocity_is_distinct_from_opening_change_and_stop(self):
        from snowbot_teleop.position import PositionOperations
        ops=PositionOperations.__new__(PositionOperations)
        ops.config=dict(calibrated=True,grippers={'left':{'closed':0.},'right':{'closed':0.,'virtual':.02}},
                        position_tolerance=.003,velocity_tolerance=.01)
        value=S(measured_position=.0006,measured_velocity=.04,state='CANCELED')
        ops.reading=lambda side:value
        self.assertEqual(ops.modes(),('unknown','unknown'))
        self.assertEqual(ops.modes(require_stationary=False),('closed','closed'))
        value.measured_position=.004
        self.assertEqual(ops.modes(require_stationary=False),('unknown','unknown'))
        value.measured_position=0.;value.state='MOVING'
        self.assertEqual(ops.modes(require_stationary=False),('unknown','unknown'))

    def test_ack_is_not_synthesized_until_matching_physics_step(self):
        from snowbot_hardware.position_sim import SimulatedGrippers
        health=dict(ready=True,stamp=10.,applied_sequences={'right_tool_axis':8},hold_positions={'arm_r_joint_8':.019})
        node=S(now=lambda:10.05,engine=S(feedback=S(stamp=10.)),physics_transport=S(health=health))
        gripper=SimulatedGrippers.__new__(SimulatedGrippers)
        gripper.node=node;gripper.core=S(cfg={'feedback_max_age':.2});gripper.pending={'right':(3,.02,True,9)}
        gripper.sample=1;gripper.acks=[0,0];gripper.holds=[0.,0.];gripper.hold_mask=0
        self.assertEqual(gripper.health()['gripper_sequence'],[0,0])
        health['applied_sequences']['right_tool_axis']=9
        actual=gripper.health()
        self.assertEqual(actual['gripper_sequence'],[0,3])
        self.assertEqual(actual['gripper_hold_position'],[0.,.019])
        self.assertEqual(actual['gripper_hold_mask'],2)
        self.assertEqual(gripper.pending,{})
        health['ready']=False
        with self.assertRaisesRegex(RuntimeError,'HEALTH_STALE'):gripper.health()

    def test_health_from_other_run_boot_profile_or_stale_time_is_ignored(self):
        from bindu_runtime.simulation_devices import SimulationDevices
        transport=SimulationDevices.__new__(SimulationDevices)
        transport.node=S(now=lambda:10.05,run_id='run');transport.profile=S(digest='profile')
        transport.simulator_id='boot';transport.health={}
        value=dict(run_id='run',runtime_mode='simulation',profile_hash='profile',simulator_id='boot',stamp=10.)
        for k,v in [('run_id','other'),('profile_hash','other'),('simulator_id','other'),('runtime_mode','real'),('stamp',9.)]:
            transport.receive_health(S(data=json.dumps(dict(value,**{k:v}))))
            self.assertEqual(transport.health,{})
        transport.receive_health(S(data=json.dumps(value)))
        self.assertEqual(transport.health,value)
