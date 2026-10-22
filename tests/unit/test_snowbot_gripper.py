"""Closed-loop gripper position/hold tests; no ROS, controller or hardware."""
from tests.support.paths import ROOT
import copy
import json
from pathlib import Path
import unittest
import ast
import asyncio
from types import SimpleNamespace as S
from bindu_contracts.profile import Profile
from bindu_contracts.teleoperation import VRFrame
from bindu_hardware.snowbot_gripper import GripperController, JOINTS, GROUPS, SIDES, validate_config, native_command

CONFIG = ROOT/'src/integration/bindu_runtime/config/robots/snowbot'


class Rig:
    def __init__(self, *, require_driver_ack=True, **changes):
        self.profile = Profile.load(CONFIG/'snowbot_real_position.json')
        self.config = json.loads((CONFIG/'snowbot_grippers_real.json').read_text())
        self.config.update(calibrated=True, **changes)
        self.now, self.steady, self.idle = 10., 100., True
        self.q = dict.fromkeys(self.profile.limits, 0.)
        self.v = dict.fromkeys(self.q, 0.)
        self.current = dict.fromkeys(SIDES, .1)
        self.health = dict(gripper_control_enabled=True, sample=1, gripper_sequence=[0,0],
            gripper_latched_mask=0, operation_enabled_mask=0xfffff, gripper_hold_mask=0, gripper_hold_position=[0.,0.])
        self.sent, self.goals = [], {}
        self.core = GripperController(self.profile, self.config, self.send, lambda:self.health,
            lambda:self.now, lambda:self.idle, enabled=True, steady=lambda:self.steady,
            require_driver_ack=require_driver_ack)
        self.observe()
        self.client_sequence = 0

    def observe(self):
        self.core.observe(self.q, self.v, self.current, self.now)

    def send(self, side, position, hold, sequence):
        self.sent.append((side,position,hold,sequence))
        i = SIDES.index(side)
        self.health['gripper_sequence'][i] = sequence
        self.goals[side] = self.q[JOINTS[side]] if hold else position
        if hold:
            self.health['gripper_hold_mask'] |= 1 << i
            self.health['gripper_hold_position'][i] = self.q[JOINTS[side]]
        else:
            self.health['gripper_hold_mask'] &= ~(1 << i)

    def acquire(self, owner='test', groups=tuple(GROUPS.values())):
        self.core.lease('acquire', owner, groups)

    def command(self, operation, side='', **kw):
        self.client_sequence += 1
        return self.core.command(operation, side, self.core.lease_id, self.core.epoch,
            self.client_sequence, self.now, .3, **kw)

    def authorize(self, load='empty', support=False):
        self.command('authorize_loading', load_state=load, external_support=support)

    def step(self, n=1, *, track=True, feedback=True, dt=.02):
        for _ in range(n):
            self.now += dt; self.steady += dt
            if track:
                for side, target in self.goals.items():
                    self.q[JOINTS[side]] = target
            if feedback:
                self.health['sample'] += 1
                self.observe()
            self.core.tick()

    def frame(self, side, seq, trigger=0., edges=(0,0), **kw):
        return VRFrame('headset', seq, self.now, side, (1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.),
            0., trigger, mapping='pico_snowbot_v1', button_edges=(0,0,0,0,0,0,*edges), **kw)


class GripperTests(unittest.TestCase):
    def test_direct_retarget_does_not_interrupt_other_channel_or_renew_its_budget(self):
        r=Rig();r.acquire();r.command('authorize_direct')
        r.command('close','left');left=dict(r.core.channels['left'])
        r.command('open','right')
        self.assertEqual(r.core.channels['left'],left)
        r.core._hold('left','SUCCEEDED','POSITION_REACHED')
        r.command('close','right')
        self.assertEqual(r.core.channels['left']['state'],'STOPPING')
        r.command('open','left')
        self.assertEqual(r.core.channels['left']['state'],'MOVING')
        r.core.stop('OPERATOR_STOP',revoke=False)
        with self.assertRaisesRegex(RuntimeError,'NOT_AUTHORIZED'):
            r.command('close','left')

    def setUp(self):
        self.r = Rig()

    def test_direct_endpoints_preserve_unknown_load_and_leave_other_side_untouched(self):
        for load in ('empty','held','unknown'):
            for side in SIDES:
                with self.subTest(load=load,side=side):
                    r=Rig();r.acquire(groups=(GROUPS[side],))
                    r.command('authorize_direct',load_state=load)
                    self.assertFalse(r.sent);self.assertEqual(r.core.load,load)
                    self.assertFalse(r.core.supported)
                    self.assertFalse(r.core.snapshot(side)['loading_authorized'])
                    r.command('open',side);r.step(20)
                    self.assertTrue(r.core.snapshot(side)['position_reached'])
                    r.command('close',side);r.step(20)
                    self.assertTrue(all(s[0]==side for s in r.sent))
                    self.assertEqual(r.core.snapshot(side)['grasp_state'],'unknown')
                    self.assertEqual(r.core.load,load)

    def test_direct_authorization_does_not_allow_arbitrary_positions_or_grasp(self):
        r=self.r;r.acquire();r.command('authorize_direct',load_state='empty')
        for operation in ('position','virtual','rigid'):
            with self.assertRaisesRegex(RuntimeError,'ENDPOINTS_ONLY|GRASP_NOT_AUTHORIZED'):
                r.command(operation,'right',position=-.2)
        self.assertFalse(r.sent)
        r.idle=False
        with self.assertRaisesRegex(RuntimeError,'STOPPED_ARMS'):r.command('open','left')
        self.assertFalse(r.sent)

    def test_direct_authorization_revoked_on_stop_and_not_reused_for_loading(self):
        r=self.r;r.acquire();r.command('authorize_direct');r.command('open','left')
        r.core.stop('TEST',revoke=False)
        self.assertFalse(r.core.direct_authorized);self.assertFalse(r.core.authorized)
        r.step(20)
        with self.assertRaisesRegex(RuntimeError,'NOT_AUTHORIZED'):r.command('open','left')
        r.authorize('held')
        with self.assertRaisesRegex(RuntimeError,'REQUIRES_SUPPORT'):r.command('open','left')

    def test_virtual_position_is_optional_and_must_be_between_closed_and_open(self):
        for value in (None, True, float('nan'), -.02, 0., .004, .4, .5):
            cfg=copy.deepcopy(self.r.config);cfg['grippers']['right']['virtual']=value
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'VIRTUAL_POSITION'):
                validate_config(cfg,self.r.profile)
        cfg=copy.deepcopy(self.r.config);del cfg['grippers']['right']['virtual']
        self.assertIs(validate_config(cfg,self.r.profile),cfg)

    def test_grasp_change_uses_only_right_axis_and_does_not_declare_contact(self):
        r=self.r;r.acquire(groups=(GROUPS['right'],));r.command('authorize_grasp')
        self.assertFalse(r.core.authorized);self.assertFalse(r.sent)
        r.command('virtual','right')
        self.assertEqual(r.sent[-1][:2],('right',.02))
        self.assertFalse(r.core.snapshot('right')['position_reached'])
        with self.assertRaises(RuntimeError):r.command('open','left')
        with self.assertRaises(RuntimeError):r.command('position','right',position=.4)
        r.step(20)
        self.assertTrue(r.core.snapshot('right')['position_reached'])
        self.assertEqual(r.core.snapshot('right')['grasp_state'],'unknown')
        self.assertEqual(r.q[JOINTS['left']],0.)
        self.assertTrue(all(packet[0]=='right' for packet in r.sent))
        token,epoch=r.core.lease_id,r.core.epoch
        r.core.lease('release',lease_id=token,epoch=epoch);r.step(12)
        self.assertEqual(r.q[JOINTS['right']],.02)
        r.acquire(groups=(GROUPS['right'],));r.command('authorize_grasp');r.command('rigid','right');r.step(20)
        self.assertTrue(r.core.snapshot('right')['position_reached'])
        self.assertEqual(r.q[JOINTS['right']],0.)

    def test_grasp_change_rejects_wrong_lease_and_open_or_moving_left_fingers(self):
        r=self.r;r.acquire()
        with self.assertRaisesRegex(RuntimeError,'AXIAL_POSITION_UNAVAILABLE'):r.command('authorize_grasp')
        r.core.lease('release',lease_id=r.core.lease_id,epoch=r.core.epoch);r.step(2)
        r.acquire(groups=(GROUPS['right'],))
        for q,v in ((-.1,0.),(0.,.1)):
            r.q[JOINTS['left']],r.v[JOINTS['left']]=q,v
            r.now+=.01;r.observe()
            with self.assertRaisesRegex(RuntimeError,'LEFT_NOT_CLOSED'):r.command('authorize_grasp')
        self.assertFalse(r.sent)

    def test_left_finger_change_during_virtual_motion_holds_right_without_closing(self):
        r=self.r;r.acquire(groups=(GROUPS['right'],));r.command('authorize_grasp');r.command('virtual','right')
        r.q[JOINTS['right']]=.01;r.q[JOINTS['left']]=-.1
        r.step(track=False)
        self.assertFalse(r.core.lease_id)
        self.assertEqual(r.sent[-1][:3],('right',.01,True))
        self.assertFalse(r.core.snapshot('right')['position_reached'])

    def test_unknown_load_requires_explicit_support_without_becoming_empty(self):
        r=self.r;r.acquire()
        with self.assertRaisesRegex(RuntimeError,'GRIPPER_LOADING_CONTEXT'):
            r.authorize('unknown',False)
        r.authorize('unknown',True)
        self.assertEqual(r.core.load,'unknown');self.assertTrue(r.core.supported)
        r.command('open','left')
        self.assertEqual(r.sent[-1][0],'left')
        self.assertEqual(r.core.snapshot('left')['grasp_state'],'unknown')

    def test_startup_readonly_uncalibrated_or_busy_never_publishes(self):
        r=self.r
        self.assertFalse(r.sent)
        for attribute,value,code in (('enabled',False,'DISABLED'),('enabled',True,'')):
            setattr(r.core,attribute,value)
            if code:
                with self.assertRaisesRegex(RuntimeError,code):r.acquire()
        r.core.cfg['calibrated']=False
        r.core.cfg['allow_uncalibrated_direct']=False
        with self.assertRaisesRegex(RuntimeError,'CALIBRATION'):r.acquire()
        r.core.cfg['calibrated']=True;r.idle=False
        with self.assertRaisesRegex(RuntimeError,'STOPPED_ARMS'):r.acquire()
        self.assertFalse(r.sent)

    def test_lease_and_authorization_do_not_move_or_confirm_grasp(self):
        r=self.r;r.acquire();r.authorize()
        self.assertFalse(r.sent)
        self.assertEqual(r.core.snapshot('left')['grasp_state'],'unknown')
        with self.assertRaisesRegex(RuntimeError,'BUSY'):r.acquire()

    def test_independent_position_requires_new_feedback_ack_and_stable_hold(self):
        r=self.r;r.acquire();r.authorize();r.command('open','left')
        self.assertEqual(r.sent[-1][0:2],('left',-.4))
        self.assertEqual(r.core.channels['left']['state'],'MOVING')
        self.assertFalse(r.core.snapshot('left')['position_reached'])
        r.step(20)
        self.assertEqual(r.core.channels['left']['state'],'SUCCEEDED')
        self.assertTrue(r.core.snapshot('left')['position_reached'])
        self.assertTrue(all(x[0]=='left' for x in r.sent))
        self.assertEqual(r.core.snapshot('left')['grasp_state'],'unknown')

    def test_already_at_target_and_same_sample_is_not_completion(self):
        r=self.r;r.acquire();r.authorize();r.command('close','left')
        for _ in range(4):r.core.tick()
        self.assertEqual(r.core.channels['left']['state'],'MOVING')

    def test_lost_driver_ack_cannot_be_hidden_by_position(self):
        r=self.r;r.acquire();r.authorize();r.command('close','left')
        r.health['gripper_sequence']=[0,0]
        r.core.send=lambda *args:None
        r.step(12)
        self.assertEqual(r.core.channels['left']['state'],'MOVING')

    def test_missing_actual_enable_cannot_report_completion(self):
        r=self.r;r.acquire();r.authorize();r.command('close','left')
        r.health['operation_enabled_mask']=0;r.step(15)
        self.assertEqual(r.core.channels['left']['state'],'MOVING')

    def test_goal_timeout_holds_and_fails_instead_of_claiming_grasp(self):
        r=Rig(motion_timeout=.3);r.acquire();r.authorize();r.command('open','right')
        r.step(30,track=False)
        self.assertEqual(r.core.channels['right']['state'],'FAILED')
        self.assertEqual(r.core.channels['right']['code'],'POSITION_TIMEOUT')
        self.assertTrue(any(x[2] for x in r.sent))

    def test_partial_hold_send_failure_still_attempts_other_side(self):
        r=self.r;r.acquire();r.authorize();r.command('open','left');r.command('open','right')
        original=r.core.send
        def broken(side,*args):
            if side=='left':raise RuntimeError('lost left endpoint')
            original(side,*args)
        r.core.send=broken;r.core.stop('cancel',revoke=True)
        self.assertTrue(any(x[0]=='right' and x[2] for x in r.sent))
        self.assertTrue(r.core.fault)

    def test_feedback_loss_latches_fault_and_blocks_reacquire(self):
        r=self.r;r.acquire();r.authorize();r.command('open','left');r.step(15,feedback=False)
        self.assertIn('STOP_UNCONFIRMED',r.core.fault)
        with self.assertRaises(RuntimeError):r.acquire()

    def test_clock_freeze_has_monotonic_feedback_budget(self):
        r=self.r;r.acquire();r.authorize();r.command('open','left')
        r.steady+=.3;r.core.tick()
        self.assertTrue(r.core.fault)

    def test_hold_timeout_is_not_extended_by_repeated_stop(self):
        r=Rig(stop_timeout=.3);r.acquire();r.authorize();r.command('open','left')
        r.core.stop('cancel',revoke=True);deadline=r.core.channels['left']['deadline']
        r.v[JOINTS['left']]=.2
        for _ in range(20):
            r.core.stop('repeat',revoke=True);r.step()
        self.assertEqual(r.core.channels['left']['deadline'],deadline)
        self.assertTrue(r.core.fault)

    def test_release_revokes_immediately_but_new_owner_waits_for_measured_hold(self):
        r=self.r;r.acquire();r.authorize();r.command('open','right')
        token,epoch=r.core.lease_id,r.core.epoch
        r.core.lease('release',lease_id=token,epoch=epoch)
        self.assertFalse(r.core.lease_id)
        with self.assertRaisesRegex(RuntimeError,'BUSY'):r.acquire()
        r.step(12);r.acquire()
        with self.assertRaisesRegex(RuntimeError,'STALE_LEASE'):r.core.check_lease(token,epoch)

    def test_limit_support_old_patch_and_malformed_health_rejected(self):
        r=self.r;r.acquire();r.authorize('held',False)
        with self.assertRaisesRegex(RuntimeError,'SUPPORT'):r.command('open','left')
        r.authorize('held',True)
        with self.assertRaisesRegex(RuntimeError,'LIMIT'):r.command('position','left',position=-100.)
        r.health['gripper_control_enabled']=False
        with self.assertRaisesRegex(RuntimeError,'PATCH'):r.command('close','left')
        r.health['gripper_control_enabled']=True;r.health['gripper_sequence']=['bad',0]
        with self.assertRaisesRegex(RuntimeError,'HEALTH_INVALID'):r.command('close','left')
        self.assertFalse(r.sent)

    def test_stale_duplicate_and_future_api_commands_rejected(self):
        r=self.r;r.acquire();r.authorize()
        for seq,stamp,ttl in ((1,r.now,.3),(2,r.now+.1,.3),(2,r.now-1.,.3),(2,r.now,1.)):
            with self.assertRaisesRegex(RuntimeError,'STALE_COMMAND'):
                r.core.command('open','left',r.core.lease_id,r.core.epoch,seq,stamp,ttl)
        self.assertFalse(r.sent)

    def test_nonfinite_or_out_of_limit_feedback_cannot_be_used_for_capture(self):
        r=self.r
        for value in (float('nan'),100.):
            r.now+=.01;r.q[JOINTS['left']]=value
            with self.assertRaises(ValueError):r.observe()
            with self.assertRaises(RuntimeError):r.acquire()
        self.assertFalse(r.sent)

    def test_position_api_cannot_bypass_loaded_opening_interlock(self):
        r=self.r;r.acquire();r.authorize('held',False)
        with self.assertRaisesRegex(RuntimeError,'POSITION_REQUIRES_SUPPORT'):
            r.command('position','left',position=.2)
        self.assertFalse(r.sent)

    def test_trigger_requires_explicit_vr_owner_authorization_and_release(self):
        r=self.r;r.acquire(owner='headset');r.authorize()
        r.core.input(r.frame('left',1,1.,(1,0)))
        self.assertFalse(r.sent)
        r.core.input(r.frame('left',2,0.,(1,1)))
        self.assertFalse(r.sent) # Initial held trigger release is not an accepted close.
        r.core.input(r.frame('left',3,1.,(2,1)))
        self.assertEqual(r.sent[-1][0:2],('left',-.4))
        r.core.input(r.frame('left',4,0.,(2,2)))
        self.assertEqual(r.sent[-1][0:2],('left',0.))

    def test_short_press_release_coalescing_never_opens_then_closes(self):
        r=self.r;r.acquire(owner='headset');r.authorize()
        r.core.input(r.frame('left',1));r.core.input(r.frame('left',2,0.,(1,1)))
        self.assertFalse(r.sent)
        self.assertFalse(r.core.lease_id)

    def test_tracking_loss_does_not_synthesize_trigger_release(self):
        r=self.r;r.acquire(owner='headset');r.authorize()
        r.core.input(r.frame('left',1));r.core.input(r.frame('left',2,1.,(1,0)))
        r.core.input(r.frame('left',3,0.,(1,0),valid=False))
        self.assertFalse(r.core.lease_id)
        self.assertTrue(r.sent[-1][2])
        self.assertNotEqual(r.core.channels['left']['state'],'SUCCEEDED')

    def test_one_hand_stream_cannot_keep_two_hand_authority_alive(self):
        r=self.r;r.acquire(owner='headset');r.authorize()
        for i in range(20):
            r.core.input(r.frame('left',i));r.step()
        self.assertFalse(r.core.lease_id)


class NativeGripperTests(unittest.TestCase):
    def rig(self):
        rig = Rig(require_driver_ack=False)
        rig.health = dict(sample=1, operation_enabled_mask=0xfffff)
        rig.acquire(); rig.authorize()
        return rig

    def test_native_packet_uses_zero_velocity_reference_and_explicit_pd(self):
        rig = self.rig()
        self.assertEqual(native_command(-.2, rig.config['grippers']['left']), [-.2, 0., 0., 100., 30.])
        with self.assertRaises(ValueError):
            native_command(float('nan'), rig.config['grippers']['left'])

    def test_native_completion_does_not_require_invented_driver_ack(self):
        rig = self.rig()
        # The native send has no sequence ACK/hold-mask response at all.
        rig.core.send = lambda side, q, hold, sequence: rig.goals.update({side: q})
        rig.command('open', 'left'); rig.step(20)
        self.assertTrue(rig.core.snapshot('left')['position_reached'])
        self.assertEqual(rig.core.snapshot('left')['grasp_state'], 'unknown')
        self.assertNotIn('gripper_sequence', rig.health)

    def test_native_hold_compares_requested_hold_against_new_measured_position(self):
        rig = self.rig()
        rig.core.send = lambda *args: None
        rig.command('close', 'left')
        for _ in range(5): rig.core.tick()
        self.assertEqual(rig.core.channels['left']['state'], 'MOVING')
        rig.step(8)
        self.assertEqual(rig.core.channels['left']['state'], 'STOPPING')
        rig.v[JOINTS['left']] = .1
        rig.step(8)
        self.assertEqual(rig.core.channels['left']['state'], 'STOPPING')
        rig.v[JOINTS['left']] = 0.
        rig.step(8)
        self.assertEqual(rig.core.channels['left']['state'], 'SUCCEEDED')


class DirectOperationTests(unittest.TestCase):
    """Run the real async coordinator and gripper core without ROS or hardware."""
    def test_uncalibrated_manual_entry_does_not_claim_tool_grasp(self):
        from bindu_teleoperation.pico import loading_guide
        path=ROOT/'src/integration/bindu_runtime/bindu_runtime/pico_node.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        method=next(n for n in cls.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='confirm_tool')
        ns={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
        mode_code='PICO_TOOL_GRASP_UNCONFIRMED_UNCALIBRATED'
        async def wait(value):return value
        node=S(feedback=lambda:S(healthy=True,stopped=True,modes=('unknown','unknown')),
            position_ops=S(config={'calibrated':False,'allow_uncalibrated_direct':True}),
            arms_ready_client=S(service_is_ready=lambda:True,call_async=lambda _:S(success=True)),
            tool_cfg={},current=lambda _:None,wait=wait)
        ns.update(time=__import__('time'),Trigger=S(Request=lambda:None))
        with self.assertRaisesRegex(RuntimeError,mode_code):
            asyncio.run(ns['confirm_tool'](node,None,'bimanual_tool'))
        asyncio.run(ns['confirm_tool'](node,None,'independent_arms'))
        guide=loading_guide(dict(phase='standby',reason='RuntimeError:'+mode_code),('unknown','unknown'))
        self.assertIn('免标定手动开合',guide['detail'])
        self.assertNotIn('尚未闭合停稳',guide['detail'])
        node.feedback=lambda:S(healthy=True,stopped=False,modes=('unknown','unknown'))
        with self.assertRaisesRegex(RuntimeError,'PICO_MODE_REQUIRES_STOPPED'):
            asyncio.run(ns['confirm_tool'](node,None,'independent_arms'))
        r=Rig(allow_uncalibrated_direct=True,require_driver_ack=False)
        r.core.cfg['calibrated']=False
        r.acquire();r.command('authorize_direct');r.command('open','left')
        r.step(150)
        self.assertEqual(r.core.snapshot('left')['grasp_state'],'unknown')
        self.assertFalse(r.core.snapshot('left')['calibrated'])

    def run_case(self, interrupt=None):
        from bindu_teleoperation.pico import PicoIntent
        r=Rig(require_driver_ack=False)
        r.q[JOINTS['right']]=.010666;r.step(track=False)
        source=ROOT/'src/integration/bindu_runtime/bindu_runtime/pico_position.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if getattr(n,'name','')=='move_grippers')
        ns=dict(time=S(monotonic=lambda:r.steady),SIDES=SIDES,GROUPS=GROUPS,json=json,
                Lease=S(Request=lambda **kw:S(**kw)),seconds=lambda value:value)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
        control=S(pending=PicoIntent(1,'manual_grippers',(None,'closed')),load='unknown',
                  supported=False,gripper_update_token=None,reason='')
        events=[];requests=[];released=[];seen_failed=False;recovered=False;cached=None
        async def wait(value):return value
        def current(intent):
            if control.pending.token!=intent.token:raise RuntimeError('SUPERSEDED')
        def lease(req):
            r.core.lease(req.operation,getattr(req,'owner',''),getattr(req,'resources',()),
                         getattr(req,'lease_id',''),getattr(req,'epoch',0))
            return S(ok=True,lease_id=r.core.lease_id,epoch=r.core.epoch)
        async def request(operation,side='',**kw):
            nonlocal cached
            if side=='right' and operation=='open':
                cached=S(stamp=r.now,**r.core.snapshot(side))
            ops.sequence+=1
            r.core.command(operation,side,ops.lease.lease_id,ops.lease.epoch,ops.sequence,
                           r.now,.25,**kw)
            requests.append((operation,side))
        def reading(side):
            nonlocal cached
            if cached is not None and side=='right':
                value,cached=cached,None;return value  # queued pre-retarget FAILED sample
            return S(stamp=r.now,**r.core.snapshot(side))
        async def release():
            released.append(r.now)
            if r.core.lease_id:r.core.lease('release',lease_id=ops.lease.lease_id,epoch=ops.lease.epoch)
        async def pause():
            nonlocal seen_failed,recovered
            elapsed=r.now-10.
            self.assertLess(elapsed,12.,'operation lost a deadline or failed to complete')
            if elapsed>=4. and control.pending.value[0] is None:
                control.pending=PicoIntent(1,'manual_grippers',('open','closed'))
            right=r.core.channels['right']
            if right['state']=='FAILED':
                seen_failed=True
                self.assertEqual(right['code'],'POSITION_TIMEOUT')
                self.assertFalse(released)
                self.assertTrue(r.core.lease_id)
                if elapsed<5.5:
                    self.assertEqual(r.core.channels['left']['state'],'MOVING')
                self.assertNotIn(('close','left'),requests)
            if elapsed>=6.5 and not recovered:
                self.assertTrue(seen_failed)
                self.assertEqual(r.core.channels['left']['state'],'SUCCEEDED')
                self.assertIn('right',control.reason)
                self.assertEqual(requests.count(('close','right')),1)  # no automatic retry
                recovered=True
                if interrupt=='stop':control.pending=PicoIntent(2,'stop')
                elif interrupt=='fault':r.core.fail('TEST_DRIVER_FAULT')
                else:control.pending=PicoIntent(1,'manual_grippers',('open','open'))
            for side,target in r.goals.items():
                # Right closure remains off-target until the operator changes it.
                if side=='right' and requests.count(('open','right'))==0:continue
                joint=JOINTS[side];delta=max(-.004,min(.004,target-r.q[joint]))
                r.q[joint]+=delta;r.v[joint]=delta/.02
            r.step(track=False)
        node=S(control=control,instance_id='offline',now=lambda:r.now,wait=wait,current=current,
               pause=pause,event=lambda kind,body:events.append((kind,json.loads(body))))
        ops=S(node=node,config=r.config,reading=reading,leases=S(call_async=lease),request=request,
              release=release,sequence=0)
        async def run():await ns['move_grippers'](ops,control.pending,direct=True)
        if interrupt:
            with self.assertRaisesRegex(RuntimeError,'SUPERSEDED' if interrupt=='stop' else 'OPERATION_FAILED'):
                asyncio.run(run())
        else:asyncio.run(run())
        self.assertTrue(seen_failed)
        self.assertEqual(len(released),1)
        self.assertIsNone(control.gripper_update_token)
        self.assertEqual([e[1]['side'] for e in events],['right'])
        return r,requests

    def test_timeout_holds_only_failed_side_and_new_input_can_reverse_it(self):
        r,requests=self.run_case()
        self.assertEqual(requests,[('authorize_direct',''),('close','right'),('open','left'),('open','right')])
        self.assertAlmostEqual(r.q[JOINTS['right']],.4)
        self.assertAlmostEqual(r.q[JOINTS['left']],-.4)

    def test_explicit_stop_cancels_pending_partial_failure(self):
        r,_=self.run_case('stop')
        self.assertFalse(r.core.lease_id)
        self.assertFalse(r.core.direct_authorized)

    def test_real_fault_still_ends_the_whole_operation(self):
        r,_=self.run_case('fault')
        self.assertTrue(r.core.fault)
        self.assertFalse(r.core.direct_authorized)


if __name__ == '__main__':
    unittest.main()
