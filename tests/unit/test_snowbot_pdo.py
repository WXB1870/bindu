"""PDO evidence, source freshness and bounded fault capture; no ROS/devices."""
import copy
from array import array
import json
import math
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import unittest
from unittest.mock import patch

from tools.teleop.snowbot_diagnostics import DiagnosticWriter
from tools.teleop.snowbot_pdo import (MASK, FaultWindow, PdoMonitor, PdoObserver,
                                       axis_status, observation_payload, statusword)


def frame(sample=1, stamp=10., **changes):
    return dict(dict(boot_id=123, pdo_sample=sample, stamp=stamp, pdo_age=.001,
                     wkc=67, expected_wkc=67, valid_mask=MASK,
                     position=[.2]*20, speed=[0.]*20, status=[0x231]*20), **changes)


class Writer:
    def __init__(self):
        self.records = []

    def record(self, stream, value):
        self.records.append((stream, copy.deepcopy(value)))

    def status(self):
        return dict(dropped=0, error='')


class PdoTests(unittest.TestCase):
    def test_historical_fault_masks_and_basic_state_decode(self):
        words = [0x231,0x218,0x218,0x231,0x218,0x231,0x218,0x218,
                 0x231,0x218,0x218,0x231,0x218,0x218,0x218,0x231,
                 0x1218,0x1218,0x1231,0x1231]
        result = axis_status(words, MASK)
        self.assertEqual(result['fault_mask'], 0x376d6)
        self.assertEqual(result['operation_enabled_mask'], 0)
        self.assertEqual(result['fault_axes'], ['L2','L3','L5','L7','L8','R2','R3','R5','R6','R7','head1','head2'])
        self.assertEqual(result['state_counts'], {'READY_TO_SWITCH_ON':8,'FAULT':12})
        words[1], words[13], words[15], words[16] = 0x231,0x231,0x218,0x1231
        self.assertEqual(axis_status(words,MASK)['fault_mask'], 0x2d6d4)

    def test_invalid_zero_status_is_unknown_and_fault_is_not_operation_enabled(self):
        result = axis_status([0]*20, 0)
        self.assertEqual(result['state_counts'], {'UNAVAILABLE':20})
        self.assertFalse(result['fault_mask_complete'])
        self.assertEqual(result['unavailable_mask'], MASK)
        result = axis_status([0x1f]*20, MASK)  # bit2 alone is insufficient.
        self.assertEqual(result['state_counts'], {'FAULT_REACTION_ACTIVE':20})
        self.assertEqual(result['operation_enabled_mask'], 0)
        self.assertEqual(axis_status([0x27]*20,MASK)['operation_enabled_mask'],MASK)
        self.assertEqual(statusword(0x1218),statusword(0x218))
        for value in (True, -1, 65536, '0x218'):
            with self.assertRaises(ValueError):statusword(value)

    def test_full_wkc_and_drive_fault_are_distinct(self):
        monitor = PdoMonitor()
        result = monitor.receive(frame(status=[0x218]*20),10.001,1.)
        self.assertTrue(result['pdo_valid'])
        self.assertEqual(result['reasons'], ['DRIVE_FAULT'])
        self.assertEqual(result['fault_mask'], MASK)
        self.assertFalse(result['specific_error_code_available'])
        self.assertFalse(result['effort_units_verified'])

    def test_repeated_sample_cannot_renew_pdo_progress(self):
        monitor = PdoMonitor()
        self.assertTrue(monitor.receive(frame(),10.001,1.)['pdo_valid'])
        self.assertTrue(monitor.receive(frame(stamp=10.01),10.011,1.01)['pdo_valid'])
        frozen = monitor.receive(frame(stamp=10.03),10.031,1.03)
        self.assertFalse(frozen['pdo_valid'])
        self.assertIn('PDO_NOT_ADVANCING',frozen['reasons'])
        self.assertTrue(monitor.receive(frame(2,10.04),10.041,1.04)['pdo_valid'])

    def test_partial_wkc_invalid_mask_and_old_measurement_cannot_look_healthy(self):
        monitor = PdoMonitor()
        failed = frame(wkc=55, valid_mask=0, pdo_age=1500., position=[float('nan')]*20,
                       speed=[float('nan')]*20, status=[0]*20)
        result = monitor.receive(failed,10.001,1.)
        self.assertFalse(result['pdo_valid'])
        self.assertIn('PDO_WKC_MISMATCH',result['reasons'])
        self.assertIn('PDO_AXIS_UNAVAILABLE',result['reasons'])
        self.assertIn('PDO_MEASUREMENT_STALE',result['reasons'])
        self.assertFalse(result['fault_mask_complete'])

    def test_combined_age_and_absent_messages_become_stale(self):
        monitor = PdoMonitor()
        result = monitor.receive(frame(stamp=10.,pdo_age=.015),10.01,1.)
        self.assertIn('PDO_MEASUREMENT_STALE',result['reasons'])
        value = frame(2,10.1)
        monitor.receive(value,10.101,1.1)
        value['stamp'] = value['pdo_age'] = -100.
        self.assertTrue(monitor.poll(10.102,1.101)['pdo_valid'])
        result = monitor.poll(10.14,1.14)
        self.assertIn('PDO_OBSERVER_STALE',result['reasons'])
        self.assertIn('PDO_NOT_ADVANCING',result['reasons'])
        self.assertFalse(result['pdo_valid'])

    def test_regression_restart_and_unverified_publisher_are_visible(self):
        monitor = PdoMonitor()
        monitor.receive(frame(20),10.001,1.)
        self.assertIn('PDO_SEQUENCE_REVERSED',monitor.receive(frame(19,10.01),10.011,1.01)['reasons'])
        restarted = monitor.receive(frame(1,10.02,boot_id=456),10.021,1.02)
        self.assertIn('SERVO_RESTART',restarted['reasons'])
        self.assertTrue(restarted['pdo_valid'])
        bad = monitor.receive(frame(2,10.03),10.031,1.03,identity_error='PDO_PUBLISHER_UNVERIFIED')
        self.assertFalse(bad['pdo_valid'])
        self.assertEqual(monitor.boot,456)
        self.assertEqual(monitor.sample,1)

    def test_observer_gaps_count_unobserved_successful_samples_without_claiming_packet_loss(self):
        monitor = PdoMonitor()
        monitor.receive(frame(10),10.001,1.)
        monitor.receive(frame(10,10.002),10.003,1.002)
        result=monitor.receive(frame(15,10.004),10.005,1.004)
        self.assertEqual(result['successful_samples_not_individually_observed'],4)
        self.assertEqual(result['repeated_samples'],1)
        self.assertEqual(result['observed_frames'],3)

    def test_malformed_fields_and_nonfinite_valid_axes_are_not_accepted(self):
        for changes in (dict(boot_id=True),dict(pdo_sample=0),dict(wkc=True),
                        dict(expected_wkc=0),dict(valid_mask=1<<20),dict(status=[0x231]*19),
                        dict(position=[float('nan')]*20),dict(speed=[True]*20),dict(stamp=float('nan'))):
            with self.subTest(changes=changes):
                self.assertFalse(PdoMonitor().receive(frame(**changes),10.001,1.)['pdo_valid'])

    def test_payload_is_detached_and_ambiguous_effort_stays_uninterpreted(self):
        raw = frame()
        raw.pop('stamp')
        msg = S(header=S(stamp=S(sec=10,nanosec=0)), current=[900.]*20, **raw)
        payload = observation_payload(msg)
        msg.position[0] = 7.
        self.assertEqual(payload['position'][0],.2)
        self.assertEqual(payload['current'][16],900.)
        self.assertNotIn('current_unit',payload)

    def test_payload_preserves_optional_hold_and_force_sample_identity_without_filling_missing_fields(self):
        raw = frame()
        raw.pop('stamp')
        msg = S(header=S(stamp=S(sec=10,nanosec=0)), session=8, command_sequence=12,
                hold_valid=False, hold_sample=23, hold_position=array('d', [.3]*16),
                wrench=array('f', [float('nan')]+[1.]*11), sensor_valid_mask=1,
                sensor_data_no=array('H', [65535, 7]), sensor_age=array('d', [.002, .5]), **raw)
        payload = observation_payload(msg)
        msg.hold_position[0] = 9.
        msg.sensor_data_no[0] = 0
        msg.sensor_age[0] = 0.
        self.assertEqual((payload['boot_id'], payload['pdo_sample'], payload['session'],
                          payload['command_sequence']), (123, 1, 8, 12))
        self.assertFalse(payload['hold_valid'])
        self.assertEqual(payload['hold_sample'], 23)
        self.assertEqual(payload['hold_position'], [.3]*16)
        self.assertEqual(payload['sensor_data_no'], [65535, 7])
        self.assertEqual(payload['sensor_age'], [.002, .5])
        self.assertEqual(payload['sensor_valid_mask'], 1)
        self.assertTrue(math.isnan(payload['wrench'][0]))
        del msg.wrench, msg.sensor_data_no, msg.sensor_age, msg.sensor_valid_mask
        missing = observation_payload(msg)
        self.assertTrue(all(name not in missing for name in ('wrench', 'sensor_data_no', 'sensor_age', 'sensor_valid_mask')))

    def test_raw_csp_evidence_keeps_integer_precision_modes_and_distinct_validity(self):
        raw = frame()
        raw.pop('stamp')
        evidence = dict(csp_diagnostics_version=1, csp_send_attempt_ns=2**60+7,
                        csp_command_mask=0xf0000, csp_feedback_mask=0,
                        csp_controlword=array('H', [0x1f]*4),
                        csp_mode_command=array('b', [8]*4), csp_mode_display=array('b', [8, 8, 9, -1]),
                        csp_target_position_raw=array('i', [2**31-1, -2**31, 123, -456]),
                        csp_target_velocity_raw=array('i', [-2**31]*4),
                        csp_target_torque_raw=array('h', [-2**15]*4),
                        csp_actual_position_raw=array('i', [321]*4),
                        csp_actual_velocity_raw=array('i', [2**31-1]*4))
        msg = S(header=S(stamp=S(sec=10, nanosec=0)), **raw, **evidence)
        payload = observation_payload(msg)
        restored = json.loads(json.dumps(payload, allow_nan=False))
        self.assertEqual(restored['csp_send_attempt_ns'], 2**60+7)
        self.assertEqual(restored['csp_target_position_raw'], [2**31-1, -2**31, 123, -456])
        self.assertEqual(restored['csp_mode_display'], [8, 8, 9, -1])
        self.assertEqual((restored['csp_command_mask'], restored['csp_feedback_mask']), (0xf0000, 0))
        self.assertEqual(restored['csp_target_torque_raw'], [-2**15]*4)
        msg.csp_target_position_raw[0] = 0
        self.assertEqual(payload['csp_target_position_raw'][0], 2**31-1)
        for name in evidence:
            delattr(msg, name)
        self.assertTrue(all(name not in observation_payload(msg) for name in evidence))

    def test_optional_servo_reference_and_output_are_copied_without_reinterpreting_or_replacing_them(self):
        raw = frame()
        raw.pop('stamp')
        arrays = ('reference_position', 'reference_speed', 'reference_acceleration',
                  'output_position', 'output_speed', 'output_torque', 'output_kp', 'output_kd')
        msg = S(header=S(stamp=S(sec=10,nanosec=0)), reference_mode=1, segment_id=91,
                sample_time_ns=2**53+7, output_valid=False, execution_mode=1,
                trajectory_phase=4, reference_valid=True, hold_basis=1,
                **{name:array('d', [float(i+1)]*16) for i, name in enumerate(arrays)}, **raw)
        payload = observation_payload(msg)
        for i, name in enumerate(arrays):
            getattr(msg, name)[0] = 99.
            self.assertEqual(payload[name], [float(i+1)]*16)
        self.assertEqual((payload['reference_mode'], payload['segment_id'], payload['sample_time_ns']),
                         (1, 91, 2**53+7))
        self.assertFalse(payload['output_valid'])
        self.assertTrue(payload['reference_valid'])
        self.assertEqual((payload['execution_mode'], payload['trajectory_phase'], payload['hold_basis']), (1, 4, 1))
        msg.reference_valid, msg.output_valid = False, True
        payload = observation_payload(msg)
        self.assertFalse(payload['reference_valid'])
        self.assertTrue(payload['output_valid'])
        del msg.execution_mode, msg.trajectory_phase, msg.reference_valid, msg.hold_basis
        missing = observation_payload(msg)
        self.assertTrue(all(name not in missing for name in
                            ('execution_mode', 'trajectory_phase', 'reference_valid', 'hold_basis')))
        self.assertEqual(payload['position'], [.2]*20)
        self.assertNotIn('q_goal', payload)
        self.assertNotIn('command_id', payload)
        self.assertNotIn('expires_at_ns', payload)


class FaultWindowTests(unittest.TestCase):
    def setUp(self):
        self.writer = Writer()
        self.window = FaultWindow(self.writer,before=2.,after=3.,capacity=4)
        self.good = dict(reasons=[],boot_id=1,wkc=67,expected_wkc=67,valid_mask=MASK,fault_mask=0)
        self.bad = dict(self.good,reasons=['DRIVE_FAULT'],fault_mask=1)

    def records(self,stream):
        return [value for key,value in self.writer.records if key==stream]

    def test_pre_and_post_window_no_duplicates_or_unbounded_fault_extension(self):
        for t in (0.,1.,2.):self.window.update(self.good,t,dict(seq=t))
        self.window.update(self.bad,3.,dict(seq=3.))
        for t in (4.,5.,6.):self.window.update(self.bad,t,dict(seq=t))
        self.window.update(self.bad,6.1,dict(seq=6.1))
        self.assertEqual([r['data']['seq'] for r in self.records('pdo-observation')],[1.,2.,3.,4.,5.,6.])
        events=self.records('pdo-events')
        self.assertEqual(events[0]['available_before_s'],2.)
        self.assertTrue(events[-1]['complete_time_window'])
        self.assertEqual(events[-1]['frames'],6)
        self.assertIsNone(self.window.until)

    def test_insufficient_history_and_early_exit_are_reported(self):
        self.window.update(self.bad,1.,dict(seq=1))
        self.window.close(2.)
        events=self.records('pdo-events')
        self.assertEqual(events[0]['available_before_s'],0.)
        self.assertFalse(events[-1]['complete_time_window'])
        self.assertEqual(events[-1]['elapsed_since_trigger_s'],1.)

    def test_capacity_and_mutation_cannot_expand_or_corrupt_history(self):
        window=FaultWindow(self.writer,before=10.,capacity=2)
        payload={'seq':1,'q':[.2]}
        window.update(self.good,1.,payload)
        payload['q'][0]=99.
        window.update(self.good,2.,dict(seq=2))
        window.update(self.bad,3.,dict(seq=3))
        self.assertEqual(len(window.ring),2)
        event=self.records('pdo-events')[0]
        self.assertEqual(event['history_evicted_at_capacity'],1)
        self.assertEqual(event['available_before_s'],1.)
        self.assertEqual([r['data']['seq'] for r in self.records('pdo-observation')],[2,3])
        for kwargs in (dict(capacity=0),dict(capacity=True),dict(before=float('nan')),dict(after=0.),
                       dict(sample_hz=0.),dict(sample_hz=float('nan'))):
            with self.assertRaises(ValueError):FaultWindow(self.writer,**kwargs)

    def test_missing_input_triggers_history_capture_without_synthetic_samples(self):
        self.window.update(self.good,1.,dict(seq=1))
        self.window.update(dict(self.good,reasons=['PDO_OBSERVER_STALE']),1.1)
        self.assertEqual([r['data']['seq'] for r in self.records('pdo-observation')],[1])
        self.window.close(1.5)
        self.assertAlmostEqual(self.records('pdo-events')[-1]['last_payload_age_s'],.5)

    def test_monotonic_reversal_closes_incomplete_window_and_discards_old_history(self):
        self.window.update(self.bad,5.,dict(seq=5))
        self.window.update(self.bad,1.,dict(seq=1))
        self.assertFalse(self.records('pdo-events')[1]['complete_time_window'])
        self.assertEqual(len(self.window.ring),1)
        self.assertEqual(self.window.event,2)

    def test_async_writer_drains_pre_and_post_data_and_keeps_original_payloads(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=DiagnosticWriter(Path(directory)/'diagnostics.jsonl',capacity=32768)
            window=FaultWindow(writer,before=.02,after=.03)
            payload={'seq':0,'q':[float('nan')]}
            window.update(self.good,0.,payload)
            payload['q'][0]=99.
            window.update(self.bad,.01,dict(seq=1))
            window.update(self.bad,.02,dict(seq=2))
            window.close(.04)
            writer.close()
            rows=[json.loads(s)['data'] for s in (Path(directory)/'pdo-observation.jsonl').read_text().splitlines()]
            self.assertEqual([r['data']['seq'] for r in rows],[0,1,2])
            self.assertEqual(rows[0]['data']['q'],['nan'])
            self.assertEqual(writer.status()['dropped'],0)
            self.assertEqual(writer.status()['error'],'')

    def test_ten_second_1khz_input_keeps_100hz_history_and_subperiod_fault_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=DiagnosticWriter(Path(directory)/'diagnostics.jsonl',capacity=32768)
            window=FaultWindow(writer)
            for sample in range(10001):
                window.update(self.good,sample*.001,dict(seq=sample))
            window.update(self.bad,10.001,dict(seq=10001))
            window.close(10.002)
            writer.close()
            rows=[json.loads(s)['data'] for s in (Path(directory)/'pdo-observation.jsonl').read_text().splitlines()]
            samples=[r['data']['seq'] for r in rows]
            self.assertEqual(samples,list(range(10,10001,10))+[10001])
            self.assertEqual(window.sampled_out,9000)
            self.assertEqual(writer.status()['dropped'],0)
            self.assertEqual(writer.status()['error'],'')

    def test_downsampling_never_hides_short_fault_or_recovery_from_the_monitor(self):
        monitor=PdoMonitor();window=FaultWindow(self.writer)
        for tick in range(100):
            packet=frame(sample=tick+1,stamp=10.+tick*.001)
            if tick==1:packet['wkc']=1
            health=monitor.receive(packet,packet['stamp'],tick*.001)
            window.update(health,tick*.001,dict(seq=tick))
        self.assertEqual(monitor.result['pdo_sample'],100)
        observed=[r['data']['seq'] for r in self.records('pdo-observation')]
        self.assertIn(1,observed)
        self.assertIn(2,observed)
        self.assertLess(len(window.ring),15)
        self.assertGreater(window.sampled_out,80)

    def test_output_validity_and_trajectory_phase_changes_bypass_window_sampling(self):
        window=FaultWindow(self.writer)
        for tick,fields in enumerate((dict(output_valid=True,trajectory_phase=1),
                dict(output_valid=False,trajectory_phase=1),dict(output_valid=False,trajectory_phase=2))):
            window.update(self.good,tick*.001,dict(seq=tick,observation=fields))
        self.assertEqual([frame['seq'] for _,frame in window.ring],[0,1,2])

    def test_healthy_pause_after_startup_window_saves_history_without_extending_on_repeats(self):
        window=FaultWindow(self.writer,before=2.,after=3.)
        window.update(self.bad,0.,dict(seq=0))
        for t in (1.,2.,3.,30.,31.,32.):window.update(self.good,t,dict(seq=t))
        self.assertIsNone(window.until)
        self.assertTrue(window.trigger(32.,'execution_stop',command_id='pause'))
        deadline=window.until
        self.assertFalse(window.trigger(34.,'execution_stop_event',code='STOP_UNCONFIRMED'))
        self.assertEqual(window.until,deadline)
        for t in (34.,35.,35.1):window.update(self.good,t,dict(seq=t))
        rows=[r for r in self.records('pdo-observation') if r['event']==2]
        self.assertEqual([r['data']['seq'] for r in rows],[30.,31.,32.,34.,35.])
        self.assertEqual(len({r['observed_monotonic'] for r in rows}),len(rows))
        self.assertTrue(self.records('pdo-events')[-1]['complete_time_window'])

    def test_external_trigger_preserves_missing_feedback_and_clock_reversal_evidence(self):
        self.window.update(self.good,5.,dict(seq=5))
        self.window.trigger(5.1,'execution_stop')
        self.window.trigger(1.,'execution_stop')
        events=self.records('pdo-events')
        self.assertFalse(events[1]['complete_time_window'])
        self.assertEqual(events[-1]['stored_pre_samples'],0)
        self.assertEqual(self.window.event,2)
        self.window.close(1.2)
        self.assertFalse(self.records('pdo-events')[-1]['complete_time_window'])


class ObserverTests(unittest.TestCase):
    def test_new_stop_near_end_of_an_old_window_receives_its_full_post_window(self):
        observer=PdoObserver.__new__(PdoObserver)
        observer.window=FaultWindow(Writer(),before=2.,after=3.)
        observer.pause_command=observer.stop_signature=None
        observer.window.update(dict(reasons=['STARTUP']),0.,dict(seq=0))
        observer.window.update(dict(reasons=[]),2.9,dict(seq=1))
        state=S(instance_id='adapter',command_id='pause-1',state='STOPPING',code='CANCELED')
        observer.execution(state,2.9)
        self.assertAlmostEqual(observer.window.until,5.9)
        observer.event(S(state='STOPPING',code='CANCELED',command_id='pause-1',task_id='',
                         component='execution',stamp=S(sec=10,nanosec=0)),3.)
        self.assertAlmostEqual(observer.window.until,5.9)
        observer.execution(S(**dict(vars(state),command_id='pause-2')),5.8)
        self.assertAlmostEqual(observer.window.until,8.8)
        self.assertEqual(observer.window.event,3)
        observer.execution(S(**dict(vars(state),command_id='pause-3')),8.8)
        ends=[value for stream,value in observer.window.writer.records
              if stream=='pdo-events' and value['kind']=='window_end']
        self.assertTrue(ends[-1]['complete_time_window'])

    def test_healthy_stop_events_capture_actual_payloads_and_terminal_state_without_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=DiagnosticWriter(Path(directory)/'diagnostics.jsonl')
            callbacks=[]
            node=S(create_subscription=lambda *args:callbacks.append(args),
                   get_publishers_info_by_topic=lambda topic:[S(node_namespace='/',node_name='servo_node',
                       topic_type='snowbot_ctl_msgs/msg/PositionObservation')])
            with patch.dict('sys.modules',{'snowbot_ctl_msgs':S(),
                    'snowbot_ctl_msgs.msg':S(PositionObservation=object),
                    'rclpy.qos':S(qos_profile_sensor_data=object())}):
                observer=PdoObserver(node,writer)
            for tick in range(4):
                raw=frame(sample=tick+1,stamp=10.+tick*.01,status=[0x27]*20)
                raw.pop('stamp')
                msg=S(header=S(stamp=S(sec=10,nanosec=tick*10_000_000)),
                      session=8,command_sequence=12,hold_valid=tick>1,hold_sample=1,
                      hold_position=[.3]*16,**raw)
                with patch('time.time',return_value=10.+tick*.01+.001),\
                        patch('time.monotonic',return_value=1.+tick*.01):
                    observer.receive(msg)
                if tick==1:
                    state=S(instance_id='adapter',command_id='pause',state='STOPPING',code='CANCELED')
                    observer.execution(state,1.011)
                    observer.execution(state,1.012)  # Repeated execution state is not a new trigger.
                    observer.event(S(state='STOPPING',code='CANCELED',command_id='pause',task_id='task',
                                     component='execution',stamp=S(sec=10,nanosec=10_000_000)),1.013)
            observer.execution(S(instance_id='adapter',command_id='pause',state='FAILED',
                                 code='STOP_UNCONFIRMED:vendor_hold'),1.04)
            observer.close(1.05);writer.close()
            rows=[json.loads(line)['data'] for line in (Path(directory)/'pdo-observation.jsonl').read_text().splitlines()]
            self.assertEqual([r['data']['observation']['pdo_sample'] for r in rows],[1,2,3,4])
            self.assertEqual([r['data']['observation']['hold_valid'] for r in rows],[False,False,True,True])
            events=[json.loads(line)['data'] for line in (Path(directory)/'pdo-events.jsonl').read_text().splitlines()]
            self.assertEqual([e['kind'] for e in events],
                             ['execution_stop','execution_stop_event','execution_stop','window_end'])
            self.assertEqual({e['event'] for e in events},{1})
            self.assertFalse(events[-1]['complete_time_window'])
            self.assertEqual(len(callbacks),1)  # No publishers, enables or device access.
            self.assertEqual(writer.status()['dropped'],0)

    def test_fault_window_keeps_hold_force_and_receive_times_in_the_same_observation(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = DiagnosticWriter(Path(directory)/'diagnostics.jsonl')
            node = S(create_subscription=lambda *args:None,
                     get_publishers_info_by_topic=lambda topic:[S(node_namespace='/',node_name='servo_node',
                         topic_type='snowbot_ctl_msgs/msg/PositionObservation')])
            with patch.dict('sys.modules', {'snowbot_ctl_msgs':S(),
                'snowbot_ctl_msgs.msg':S(PositionObservation=object),
                'rclpy.qos':S(qos_profile_sensor_data=object())}):
                observer = PdoObserver(node, writer)
            raw = frame(status=[0x218]*20)
            raw.pop('stamp')
            msg = S(header=S(stamp=S(sec=10,nanosec=0)), session=8, command_sequence=12,
                    hold_valid=True, hold_sample=1, hold_position=array('d', [.3]*16),
                    wrench=array('f', [float('nan')]+[1.]*11), sensor_valid_mask=1,
                    sensor_data_no=array('H', [10, 20]), sensor_age=array('d', [.002, .5]), **raw)
            with patch('time.time',return_value=10.003),patch('time.monotonic',return_value=1.):
                observer.receive(msg)
            observer.close(1.01)
            writer.close()
            row = json.loads((Path(directory)/'pdo-observation.jsonl').read_text())
            observed, payload = row['data']['data'], row['data']['data']['observation']
            self.assertEqual((observed['received_ros'], observed['received_monotonic']), (10.003, 1.))
            self.assertEqual((payload['stamp'], payload['pdo_age'], payload['boot_id'], payload['pdo_sample']),
                             (10., .001, 123, 1))
            self.assertEqual((payload['session'], payload['command_sequence'], payload['hold_sample']), (8, 12, 1))
            self.assertEqual(payload['hold_position'], [.3]*16)
            self.assertEqual(payload['sensor_data_no'], [10, 20])
            self.assertEqual(payload['sensor_age'], [.002, .5])
            self.assertEqual(payload['wrench'][0], 'nan')
            self.assertNotIn('reference_position', payload)
            self.assertNotIn('output_position', payload)
            self.assertNotIn('output_valid', payload)
            self.assertEqual(writer.status()['dropped'], 0)
            self.assertEqual(writer.status()['error'], '')

    def test_observer_only_subscribes_and_reports_unverified_publishers(self):
        callbacks = []
        queries = []
        info = S(node_namespace='/',node_name='servo_node',topic_type='snowbot_ctl_msgs/msg/PositionObservation')
        infos = [info]
        def publishers(topic):
            queries.append(topic)
            return infos
        node = S(create_subscription=lambda kind, topic, cb, qos:callbacks.append((topic,cb)),
                 get_publishers_info_by_topic=publishers)
        with patch.dict('sys.modules',{'snowbot_ctl_msgs':S(),
            'snowbot_ctl_msgs.msg':S(PositionObservation=object),
            'rclpy.qos':S(qos_profile_sensor_data=object())}):
            observer = PdoObserver(node)
        self.assertEqual([topic for topic, _ in callbacks],['/bindu/position/observation'])
        raw=frame();raw.pop('stamp')
        msg=S(header=S(stamp=S(sec=10,nanosec=0)),**raw)
        def receive(steady, metadata=None):
            msg.header.stamp.nanosec = round((steady-1.)*1e9)
            msg.pdo_sample += 1
            with patch('time.time',return_value=steady+9.001),patch('time.monotonic',return_value=steady):
                callbacks[0][1](msg,metadata)
            return observer.sample(steady+9.002,steady+.001)
        self.assertTrue(receive(1.)['pdo_valid'])
        self.assertTrue(receive(1.002)['pdo_valid'])
        self.assertEqual(len(queries),1)  # Discovery is not queried per frame.
        infos = [info,info]
        self.assertIn('PDO_PUBLISHER_UNVERIFIED',receive(1.204)['reasons'])
        infos = [info]
        info.endpoint_gid=[0]*16
        self.assertIn('PDO_PUBLISHER_GID_MISMATCH',receive(1.408,S(publisher_gid=[1]*16))['reasons'])
        self.assertEqual(receive(1.410,S(publisher_gid=[0]*16))['publisher_gid_verification'],'verified')
        self.assertIn('PDO_PUBLISHER_GID_MISMATCH',receive(1.412,S(publisher_gid=[1]*16))['reasons'])
        self.assertEqual(len(queries),3)  # Cached graph still checks each callback GID.
        infos = [S(**dict(vars(info),endpoint_gid=[2]*24))]
        self.assertEqual(receive(1.612,{'publisher_gid':[2]*24})['publisher_gid_verification'],'verified')

    def test_missing_optional_interface_is_visible_without_starting_ros_or_hardware(self):
        # This interface is optional on the Mac and on an original vendor install.
        with patch.dict('sys.modules',{'snowbot_ctl_msgs':None,'snowbot_ctl_msgs.msg':None}):
            observer=PdoObserver(S())
        state=observer.sample(10.,1.)
        self.assertFalse(state['available'])
        self.assertEqual(state['reasons'],['PDO_OBSERVATION_TYPE_UNAVAILABLE'])
        observer.close(1.)


if __name__ == '__main__':
    unittest.main()
