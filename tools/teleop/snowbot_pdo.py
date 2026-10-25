"""Read-only PDO diagnostics in the display process; no ROS or device commands."""
from collections import Counter, deque
from collections.abc import Mapping
import copy
import math


AXES = tuple([*(f'L{i}' for i in range(1, 9)), *(f'R{i}' for i in range(1, 9)),
              'head1', 'head2', 'torso1', 'torso2'])
MASK = (1 << len(AXES)) - 1


def integer(value, minimum=0, maximum=2**64-1):
    return type(value) is int and minimum <= value <= maximum


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def statusword(value):
    """Decode only CiA402 basic state bits; manufacturer/mode high bits stay raw."""
    if not integer(value, maximum=0xffff):
        raise ValueError('PDO_STATUSWORD_INVALID')
    if value & 0x4f == 0x08:
        return 'FAULT'
    if value & 0x4f == 0x0f:
        return 'FAULT_REACTION_ACTIVE'
    if value & 0x4f == 0:
        return 'NOT_READY'
    if value & 0x4f == 0x40:
        return 'SWITCH_ON_DISABLED'
    return {0x21: 'READY_TO_SWITCH_ON', 0x23: 'SWITCHED_ON',
            0x27: 'OPERATION_ENABLED', 0x07: 'QUICK_STOP_ACTIVE'}.get(value & 0x6f, 'UNKNOWN')


def axis_status(words, valid_mask):
    """Invalid axes are unknown, including a zero substituted by a publisher."""
    if len(words) != 20 or not integer(valid_mask, maximum=MASK):
        raise ValueError('PDO_STATUS_LAYOUT')
    states, faults, enabled = {}, 0, 0
    for i, (name, word) in enumerate(zip(AXES, words)):
        state = statusword(word)
        if not valid_mask & (1 << i):
            state = 'UNAVAILABLE'
        else:
            if word & 8:
                faults |= 1 << i
            if state == 'OPERATION_ENABLED':
                enabled |= 1 << i
        states[name] = state
    return dict(states=states, fault_mask=faults, fault_axes=[AXES[i] for i in range(20) if faults & (1 << i)],
                fault_mask_complete=valid_mask == MASK, operation_enabled_mask=enabled,
                unavailable_mask=MASK ^ valid_mask, state_counts=dict(Counter(states.values())))


def observation_payload(msg):
    """Detach the supplemental message once, outside the servo/RT processes."""
    value = dict(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
    for name in ('boot_id', 'pdo_sample', 'pdo_age', 'wkc', 'expected_wkc', 'valid_mask'):
        value[name] = getattr(msg, name)
    for name in ('position', 'speed', 'status'):
        item = getattr(msg, name)
        # ROS fixed arrays may be NumPy arrays; list() keeps their scalar types.
        value[name] = item.tolist() if hasattr(item, 'tolist') else list(item)
    # Preserve optional evidence without assigning physical units to an ambiguous field.
    arrays = ('current', 'torque', 'torque_raw', 'hold_position', 'wrench', 'sensor_data_no', 'sensor_age',
              'reference_position', 'reference_speed', 'reference_acceleration',
              'output_position', 'output_speed', 'output_torque', 'output_kp', 'output_kd',
              'csp_controlword', 'csp_mode_command', 'csp_mode_display',
              'csp_target_position_raw', 'csp_target_velocity_raw', 'csp_target_torque_raw',
              'csp_actual_position_raw', 'csp_actual_velocity_raw')
    for name in (*arrays, 'current_valid_mask', 'torque_valid_mask',
                 'torque_raw_valid_mask', 'receive_attempt', 'attempt_age', 'bad_wkc_count',
                 'receive_overruns', 'period_overruns', 'command_sequence',
                 'session', 'session_reason', 'session_selected', 'hold_valid', 'hold_sample',
                 'sensor_valid_mask', 'reference_mode', 'segment_id', 'sample_time_ns', 'output_valid',
                 'execution_mode', 'trajectory_phase', 'reference_valid', 'hold_basis',
                 'csp_diagnostics_version', 'csp_send_attempt_ns', 'csp_command_mask', 'csp_feedback_mask'):
        if hasattr(msg, name):
            item = getattr(msg, name)
            value[name] = (item.tolist() if hasattr(item, 'tolist') else list(item)) if name in arrays else item
    return value


class PdoMonitor:
    """Diagnostic validity only. This class never grants control readiness."""
    def __init__(self, max_age=.02):
        self.max_age = max_age
        self.boot = self.sample = None
        self.progress = self.received = None
        self.measurement_time = None
        self.frames = self.repeats = self.unobserved = self.regressions = self.restarts = 0
        self.result = dict(available=False, pdo_valid=False, reasons=['WAITING_FOR_PDO'])

    def receive(self, value, ros_now, steady_now, *, identity_error=''):
        reasons = []
        result = dict(available=True, pdo_valid=False, stamp=ros_now,
                      reasons=reasons, observation_only=True, specific_error_code_available=False,
                      effort_units_verified=False)
        try:
            if not all(number(x) for x in (ros_now, steady_now, value['stamp'], value['pdo_age'])):
                raise ValueError('PDO_TIME_INVALID')
            if not all(integer(value[k], 1) for k in ('boot_id', 'pdo_sample')):
                raise ValueError('PDO_IDENTITY_INVALID')
            if not integer(value['wkc'], maximum=2**31-1) or not integer(value['expected_wkc'], 1, 2**31-1):
                raise ValueError('PDO_WKC_INVALID')
            decoded = axis_status(value['status'], value['valid_mask'])
            if any(len(value[k]) != 20 for k in ('position', 'speed')):
                raise ValueError('PDO_MEASUREMENT_LAYOUT')
            for i in range(20):
                if value['valid_mask'] & (1 << i) and not all(number(value[k][i]) for k in ('position', 'speed')):
                    raise ValueError('PDO_NONFINITE_VALID_MEASUREMENT')
            result.update(decoded, boot_id=value['boot_id'], pdo_sample=value['pdo_sample'],
                          wkc=value['wkc'], expected_wkc=value['expected_wkc'], valid_mask=value['valid_mask'],
                          pdo_age_s=value['pdo_age'], publication_age_s=ros_now-value['stamp'])
            if identity_error:
                reasons.append(identity_error)
            if self.received is not None and steady_now < self.received:
                reasons.append('OBSERVER_CLOCK_REVERSED')
            if not identity_error:
                if self.boot is not None and value['boot_id'] != self.boot:
                    reasons.append('SERVO_RESTART')
                    self.sample = self.progress = None
                    self.restarts += 1
                if self.sample is not None and value['pdo_sample'] < self.sample:
                    reasons.append('PDO_SEQUENCE_REVERSED')
                    self.regressions += 1
                elif self.sample is None or value['pdo_sample'] > self.sample:
                    if self.sample is not None:
                        self.unobserved += value['pdo_sample']-self.sample-1
                    self.sample, self.progress = value['pdo_sample'], steady_now
                else:
                    self.repeats += 1
                self.boot = value['boot_id']
                self.frames += 1
            result.update(observed_frames=self.frames, repeated_samples=self.repeats,
                          successful_samples_not_individually_observed=self.unobserved,
                          sequence_regressions=self.regressions, servo_restarts=self.restarts)
            if value['wkc'] != value['expected_wkc']:
                reasons.append('PDO_WKC_MISMATCH')
            if value['valid_mask'] != MASK:
                reasons.append('PDO_AXIS_UNAVAILABLE')
            if decoded['fault_mask']:
                reasons.append('DRIVE_FAULT')
            self.measurement_time = (value['stamp'], value['pdo_age'])
        except (KeyError, TypeError, ValueError) as exc:
            reasons.append(str(exc))
            self.measurement_time = None
        self.received = steady_now
        self.result = result
        return self.poll(ros_now, steady_now)

    def poll(self, ros_now, steady_now):
        result = copy.deepcopy(self.result)
        reasons = result['reasons']
        if self.received is not None:
            elapsed = steady_now-self.received
            if not 0 <= elapsed <= self.max_age:
                reasons.append('PDO_OBSERVER_STALE')
            if self.progress is None or not 0 <= steady_now-self.progress <= self.max_age:
                reasons.append('PDO_NOT_ADVANCING')
            if self.measurement_time is not None:
                stamp, pdo_age = self.measurement_time
                age = ros_now-stamp + pdo_age
                result['measurement_age_s'] = age
                if (not 0 <= pdo_age <= self.max_age or
                        not 0 <= ros_now-stamp <= self.max_age or
                        not 0 <= age <= self.max_age):
                    reasons.append('PDO_MEASUREMENT_STALE')
        result['reasons'] = list(dict.fromkeys(reasons))
        # Drive faults do not invalidate an otherwise fresh, complete PDO receive.
        bus_reasons = set(result['reasons']) - {'DRIVE_FAULT', 'SERVO_RESTART'}
        result['pdo_valid'] = bool(self.measurement_time is not None and not bus_reasons)
        result['checked_at'] = ros_now
        return result


class FaultWindow:
    """Bounded prehistory; enqueue selected frames to the existing async writer.

    Repeated faults cannot extend a window forever. The observer, not the RT
    thread, calls this class. Queue loss remains visible in DiagnosticWriter.
    """
    def __init__(self, writer, *, before=10., after=30., capacity=1200, sample_hz=100.):
        if (not all(number(x) and x > 0 for x in (before, after, sample_hz)) or
                not integer(capacity, 1, 100000)):
            raise ValueError('PDO_WINDOW_BOUNDS')
        self.writer, self.before, self.after, self.capacity = writer, before, after, capacity
        self.ring = deque()
        self.until = None
        self.triggered_at = None
        self.signature = None
        self.event = self.frames = self.evicted = 0
        self.last_time = None
        self.last_payload = None
        self.last_retained = None
        self.payload_signature = None
        self.sample_hz, self.sampled_out = sample_hz, 0

    def _finish(self, now, complete):
        if self.until is not None:
            self.writer.record('pdo-events', dict(kind='window_end', event=self.event, complete_time_window=complete,
                elapsed_since_trigger_s=max(0., min(self.after, now-self.triggered_at)),
                last_payload_age_s=now-self.last_payload if self.last_payload is not None else None,
                frames=self.frames, sample_hz=self.sample_hz, sampled_out=self.sampled_out,
                writer=self.writer.status()))
            self.until = self.triggered_at = None

    def _advance(self, now):
        if not number(now):
            raise ValueError('PDO_WINDOW_CLOCK')
        if self.last_time is not None and now < self.last_time:
            self._finish(self.last_time, False)
            self.ring.clear()
            self.signature = None
            self.last_retained = None
            self.payload_signature = None
        self.last_time = now
        if self.until is not None and now > self.until:
            self._finish(self.until, True)
        while self.ring and self.ring[0][0] < now-self.before:
            self.ring.popleft()

    def trigger(self, now, kind, *, restart=False, **details):
        """Share one bounded window across bus faults and execution stops."""
        self._advance(now)
        if restart:
            self._finish(now, self.until is not None and now >= self.until)
        starting = self.until is None
        if starting:
            self.event += 1
            self.frames = 0
            self.triggered_at, self.until = now, now+self.after
        self.writer.record('pdo-events', dict(kind=kind, event=self.event,
            requested_before_s=self.before, requested_after_s=self.after,
            available_before_s=now-self.ring[0][0] if self.ring else 0.,
            history_evicted_at_capacity=self.evicted, stored_pre_samples=len(self.ring),
            sample_hz=self.sample_hz, sampled_out=self.sampled_out, **details))
        if starting:
            for stamp, frame in self.ring:
                self.writer.record('pdo-observation', dict(event=self.event, observed_monotonic=stamp, data=frame))
                self.frames += 1
        return starting

    def update(self, health, now, payload=None):
        self._advance(now)
        signature = (tuple(health['reasons']), health.get('boot_id'), health.get('wkc'),
                     health.get('expected_wkc'), health.get('valid_mask'), health.get('fault_mask'),
                     health.get('operation_enabled_mask'))
        changed = signature != self.signature
        self.signature = signature
        retain = False
        if payload is not None:
            self.last_payload = now
            observation = payload.get('observation', payload)
            payload_signature = tuple(observation.get(name) for name in
                ('reference_mode', 'execution_mode', 'trajectory_phase', 'reference_valid',
                 'output_valid', 'hold_valid', 'hold_basis'))
            payload_changed = payload_signature != self.payload_signature
            self.payload_signature = payload_signature
            retain = (changed or payload_changed or self.last_retained is None or
                      now-self.last_retained+1e-12 >= 1/self.sample_hz)
        if retain:
            self.last_retained = now
            if len(self.ring) == self.capacity:
                self.ring.popleft()
                self.evicted += 1
            self.ring.append((now, copy.deepcopy(payload)))
        elif payload is not None:
            self.sampled_out += 1
        if changed and health['reasons']:
            if self.trigger(now, 'fault_transition', health=health):
                return
        if self.until is not None and retain:
            self.writer.record('pdo-observation', dict(event=self.event, observed_monotonic=now, data=self.ring[-1][1]))
            self.frames += 1

    def close(self, now):
        self._finish(now, self.until is not None and now >= self.until)


class PdoObserver:
    """Subscribe only to the supplemental interface; absence stays explicit."""
    def __init__(self, node, writer=None):
        self.node, self.writer = node, writer
        self.monitor = PdoMonitor()
        self.window = FaultWindow(writer) if writer else None
        self.last_summary = None
        self.publishers, self.graph_checked = (), None
        self.stop_signature = None
        self.pause_command = None
        self.unavailable = ''
        try:
            from snowbot_ctl_msgs.msg import PositionObservation
            from rclpy.qos import qos_profile_sensor_data
        except ImportError:
            self.unavailable = 'PDO_OBSERVATION_TYPE_UNAVAILABLE'
            return
        self.subscription = node.create_subscription(PositionObservation, '/bindu/position/observation',
                                                      self.receive, qos_profile_sensor_data)

    def receive(self, msg, metadata=None):
        import time
        now, steady = time.time(), time.monotonic()
        identity_error = ''
        # Discovery is diagnostic only; cache for 200 ms, but compare GID on every frame.
        if self.graph_checked is None or not 0 <= steady-self.graph_checked < .2:
            self.publishers = tuple(self.node.get_publishers_info_by_topic('/bindu/position/observation'))
            self.graph_checked = steady
        infos = self.publishers
        if (len(infos) != 1 or infos[0].node_namespace.rstrip('/')+'/'+infos[0].node_name != '/servo_node' or
                infos[0].topic_type != 'snowbot_ctl_msgs/msg/PositionObservation'):
            identity_error = 'PDO_PUBLISHER_UNVERIFIED'
        gid_state = 'unavailable'
        received_gid = None
        gid = metadata.get('publisher_gid') if isinstance(metadata, Mapping) else getattr(metadata, 'publisher_gid', None)
        if gid is not None:
            try:
                received_gid = bytes(gid)
                expected_gid = bytes(infos[0].endpoint_gid) if len(infos) == 1 else b''
                if not received_gid or received_gid != expected_gid:
                    identity_error, gid_state = 'PDO_PUBLISHER_GID_MISMATCH', 'mismatch'
                else:
                    gid_state = 'verified'
            except (AttributeError, TypeError, ValueError):
                identity_error, gid_state = 'PDO_PUBLISHER_GID_UNVERIFIED', 'unavailable'
        try:
            payload = observation_payload(msg)
        except (AttributeError, TypeError, ValueError) as exc:
            payload = dict(malformed=type(exc).__name__)
        result = self.monitor.receive(payload, now, steady, identity_error=identity_error)
        result['publisher_gid_verification'] = gid_state
        self.monitor.result['publisher_gid_verification'] = gid_state
        if self.window:
            self.window.update(result, steady, dict(received_ros=now, received_monotonic=steady,
                publisher_graph_verified=not identity_error, publisher_gid=received_gid.hex() if received_gid else None,
                publisher_gid_verification=gid_state, observation=payload))

    def sample(self, now, steady):
        result = self.monitor.poll(now, steady)
        if self.unavailable:
            result.update(available=False, pdo_valid=False, reasons=[self.unavailable])
        if self.window and result.get('available'):
            self.window.update(result, steady)
        if self.window:
            result.update(log_sample_hz=self.window.sample_hz, log_sampled_out=self.window.sampled_out)
        if self.writer and (self.last_summary is None or steady-self.last_summary >= 1.):
            self.writer.record('pdo-health', result)
            self.last_summary = steady
        return result

    def execution(self, msg, steady):
        """Execution errors need capture even when WKC and drives stay healthy."""
        signature = (msg.instance_id, msg.command_id, msg.state, msg.code)
        if signature == self.stop_signature:
            return
        self.stop_signature = signature
        if self.window and (msg.state == 'STOPPING' or 'STOP_UNCONFIRMED' in msg.code):
            self._capture_stop(steady, 'execution_stop', state=msg.state, code=msg.code,
                               instance_id=msg.instance_id, command_id=msg.command_id)

    def event(self, msg, steady):
        if self.window and (msg.state in ('STOPPING', 'BIMANUAL_PAUSE_REQUESTED') or
                            'STOP_UNCONFIRMED' in msg.code):
            self._capture_stop(steady, 'execution_stop_event', state=msg.state, code=msg.code,
                               command_id=msg.command_id, task_id=msg.task_id,
                               component=msg.component,
                               source_stamp_ns=msg.stamp.sec*1_000_000_000+msg.stamp.nanosec)

    def _capture_stop(self, steady, kind, **details):
        command = details['command_id']
        new_pause = (details['state'] in ('STOPPING', 'BIMANUAL_PAUSE_REQUESTED') and
                     bool(command) and command != self.pause_command)
        if new_pause:
            self.pause_command = command
        # A new PAUSE needs its full post-window, even near the end of an old
        # startup/fault window. Repeated reports of that stop never extend it.
        self.window.trigger(steady, kind, restart=new_pause, **details)

    def close(self, steady):
        if self.window:
            self.window.close(steady)
