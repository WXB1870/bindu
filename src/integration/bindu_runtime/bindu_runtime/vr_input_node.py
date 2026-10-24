"""Explicitly launched Vuer input process; no hardware output."""
import time
import json
import multiprocessing as mp
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from bindu_teleoperation.vr.receiver import VuerReceiver, _latest_put
from bindu_interfaces.msg import VRInput
from .common import RuntimeNode, stamp, spin


def _publish_health(topic, reports):
    """DDS diagnostic backpressure must not hold the input polling interpreter."""
    import rclpy
    rclpy.init(args=[])
    node=rclpy.create_node('vr_diagnostics')
    publisher=node.create_publisher(String,topic,10)
    previous_publish=0.
    try:
        while True:
            report=reports.get()
            if report is None:break
            report['health_sink_queue_age_s']=time.time()-report.pop('health_queued_at')
            report['previous_health_publish_wall_s']=previous_publish
            started=time.monotonic()
            publisher.publish(String(data=json.dumps(report)))
            previous_publish=time.monotonic()-started
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node();rclpy.try_shutdown()


class VRInputNode(RuntimeNode):
    def __init__(self):
        super().__init__('vr_input', allow_hardware=True, default_mode='simulation')
        defaults = {'host': '127.0.0.1', 'port': 8012, 'side': 'left', 'cert_file': '', 'key_file': '', 'mapping': 'v34'}
        for key, value in defaults.items():
            self.declare_parameter(key, value, descriptor=ParameterDescriptor(read_only=key=='mapping'))
        cfg = {key: self.get_parameter(key).value for key in defaults}
        self.declare_parameter('input_timing_enabled',False,descriptor=ParameterDescriptor(read_only=True))
        cfg['input_timing_enabled']=self.get_parameter('input_timing_enabled').value
        if type(cfg['input_timing_enabled']) is not bool:raise ValueError('VR_INPUT_TIMING_CONFIG')
        from bindu_teleoperation.vr.timing import InputPipelineTiming
        self.input_timing=InputPipelineTiming() if cfg['input_timing_enabled'] else None
        self.declare_parameter('simulation_demo',False)
        demo=self.get_parameter('simulation_demo').value
        if type(demo) is not bool or (demo and (not self.simulation or cfg['mapping']!='pico_snowbot_v1')):
            raise ValueError('VR_DEMO_REQUIRES_SIMULATION')
        cfg['simulation_demo']=demo
        if cfg['side'] not in ('left', 'right', 'both'):
            raise ValueError('VR_INVALID_SIDE')
        cfg['cert_file'] = cfg['cert_file'] or None
        cfg['key_file'] = cfg['key_file'] or None
        self.mapping=cfg['mapping']
        if self.mapping=='pico_snowbot_v1' and self.get_parameter('robot_config').value:
            from .robot_setup import RobotSetup
            robot=RobotSetup.load(self.get_parameter('robot_config').value)
            cfg.update(display_urdf=str(robot.display_urdf),display_tool_config=str(robot.tool_config))
        self.receiver = VuerReceiver({**cfg,'runtime_mode':self.runtime_mode})
        # Both sides share one receiver event; retain both messages until DDS
        # delivers the pair to mode, single-arm and bimanual subscribers.
        self.publisher = self.create_publisher(VRInput, 'teleop/vr/input', 5)
        ctx=mp.get_context('spawn')
        self.health_reports=ctx.Queue(maxsize=256)
        self.health_dropped=0
        self.health_failed=False
        self.health_process=ctx.Process(target=_publish_health,
            args=(self.resolve_topic_name('teleop/vr/health'),self.health_reports),daemon=True)
        self.health_process.start()
        self.last_frame = None
        self.input_clock_offset = None
        self.last_poll = None
        self.poll_delay_max = 0.
        self.timer = self.create_timer(.005, self.poll)
        self.create_subscription(String, 'teleop/state' if self.mapping=='pico_snowbot_v1' else 'teleop/display', self.on_display,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_display(self, msg):
        try:
            view = json.loads(msg.data)
            if (view['run_id'] == self.run_id and view['profile_hash'] == self.profile.digest
                    and view.get('runtime_mode') == self.runtime_mode
                    and 0 <= self.now()-view['stamp'] <= .6):
                if self.mapping=='pico_snowbot_v1':
                    view=dict(view,pico=True,mode=view['phase'],target=None,measured=None,error=None)
                self.receiver.display(view)
        except (ValueError, KeyError, TypeError):
            pass

    def emit_health(self, report):
        if not self.health_process.is_alive() and not self.health_failed:
            self.health_failed=True
            self.get_logger().error('VR_DIAGNOSTICS_EXITED: input polling continues')
        report.update(health_queued_at=time.time(),health_delivery_dropped=self.health_dropped)
        if _latest_put(self.health_reports,report):self.health_dropped+=1

    def poll(self):
        timing=getattr(self,'input_timing',None)
        if timing is None:return VRInputNode.poll_input(self)
        self.input_publish_count=0
        timing.begin(time.monotonic())
        try:VRInputNode.poll_input(self)
        finally:
            report=timing.finish(time.monotonic(),getattr(self.receiver,'last_delivery',None),self.input_publish_count)
            if report:self.emit_health(dict(report,run_id=self.run_id,runtime_mode=self.runtime_mode,
                                            stamp=time.time()))

    def poll_input(self):
        polled=time.monotonic()
        if self.last_poll is not None:self.poll_delay_max=max(self.poll_delay_max,polled-self.last_poll)
        self.last_poll=polled
        for report in self.receiver.poll_health():
            report.update(run_id=self.run_id,runtime_mode=self.runtime_mode,
                          ros_poll_interval_max_s=self.poll_delay_max,ros_received_at=time.time(),
                          ros_received_monotonic_s=time.monotonic())
            self.emit_health(report)
            self.poll_delay_max=0.
        try:
            frame = self.receiver.poll()
        except RuntimeError as exc:
            self.event('VR_INPUT_FAILED', str(exc))
            self.get_logger().error(str(exc))
            self.timer.cancel()
            return
        if frame is None:
            return
        first=frame[0] if isinstance(frame,tuple) else frame
        if self.last_frame and self.last_frame.source_id==first.source_id:
            gap=first.stamp-self.last_frame.stamp
            if gap>.1 or first.seq-self.last_frame.seq>1:
                self.emit_health(dict(
                    run_id=self.run_id,runtime_mode=self.runtime_mode,stage='ros_input_publish',
                    stamp=time.time(),source_id=first.source_id,seq=first.seq,
                    event_interval_s=gap,skipped_events=first.seq-self.last_frame.seq-1,
                    receiver_to_ros_age_s=time.time()-first.stamp))
        self.last_frame=first
        for item in frame if isinstance(frame, tuple) else (frame,):
            age = time.time()-item.stamp
            if age < 0 or age > 1.:
                continue
            if self.input_clock_offset is None:
                before = time.time()
                node_time = self.now()
                after = time.time()
                self.input_clock_offset = node_time-(before+after)/2
            # Map the receiver clock once for this node. Re-estimating the
            # offset per hand/event turns scheduler/DDS delay into timestamp
            # skew and can reverse two closely spaced received packets.
            # Preserve source intervals; never clamp or refresh old input.
            started=time.monotonic()
            self.publisher.publish(VRInput(stamp=stamp(item.stamp+self.input_clock_offset), source_id=item.source_id,
                seq=item.seq, side=item.side, runtime_mode=self.runtime_mode,
                pose=list(item.pose) if item.valid else [0.]*16,
                grip=item.grip, trigger=item.trigger, init=item.init, stop=item.stop,
                valid=item.valid, clutch_seq=item.clutch_seq, mapping=item.mapping,
                buttons=item.buttons, button_edges=list(item.button_edges), virtual_grip=item.virtual_grip,
                stick=list(item.stick),head_pose=list(item.head_pose),
                head_stamp=stamp(item.head_stamp+self.input_clock_offset) if item.head_pose else stamp(0.),
                head_age_s=item.head_age_s,head_source_id=item.head_source_id,head_seq=item.head_seq))
            if getattr(self,'input_timing',None) is not None:self.input_publish_count+=1
            duration=time.monotonic()-started
            if duration>.02:
                self.emit_health(dict(stage='ros_input_publish_duration',stamp=time.time(),
                    run_id=self.run_id,runtime_mode=self.runtime_mode,seq=item.seq,
                    side=item.side,publish_wall_s=duration))

    def destroy_node(self):
        self.receiver.close()
        _latest_put(self.health_reports,None)
        self.health_process.join(timeout=2.)
        if self.health_process.is_alive():
            self.health_process.terminate();self.health_process.join(timeout=1.)
        self.health_reports.cancel_join_thread()
        self.health_reports.close()
        return super().destroy_node()


def main():
    spin(VRInputNode, threaded=False)
