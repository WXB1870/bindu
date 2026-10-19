"""Explicitly launched Vuer input process; no hardware output."""
import time
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from bindu_teleoperation.vr.receiver import VuerReceiver
from bindu_interfaces.msg import VRInput
from .common import RuntimeNode, stamp, spin


class VRInputNode(RuntimeNode):
    def __init__(self):
        super().__init__('vr_input', allow_hardware=True, default_mode='simulation')
        defaults = {'host': '127.0.0.1', 'port': 8012, 'side': 'left', 'cert_file': '', 'key_file': '', 'mapping': 'v34'}
        for key, value in defaults.items():
            self.declare_parameter(key, value, descriptor=ParameterDescriptor(read_only=key=='mapping'))
        cfg = {key: self.get_parameter(key).value for key in defaults}
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
        self.health_publisher = self.create_publisher(String,'teleop/vr/health',10)
        self.last_frame = None
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

    def poll(self):
        polled=time.monotonic()
        if self.last_poll is not None:self.poll_delay_max=max(self.poll_delay_max,polled-self.last_poll)
        self.last_poll=polled
        for report in self.receiver.poll_health():
            report.update(run_id=self.run_id,runtime_mode=self.runtime_mode,
                          ros_poll_interval_max_s=self.poll_delay_max,ros_received_at=time.time())
            self.health_publisher.publish(String(data=json.dumps(report)))
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
                self.health_publisher.publish(String(data=json.dumps(dict(
                    run_id=self.run_id,runtime_mode=self.runtime_mode,stage='ros_input_publish',
                    stamp=time.time(),source_id=first.source_id,seq=first.seq,
                    event_interval_s=gap,skipped_events=first.seq-self.last_frame.seq-1,
                    receiver_to_ros_age_s=time.time()-first.stamp))))
        self.last_frame=first
        for item in frame if isinstance(frame, tuple) else (frame,):
            age = time.time()-item.stamp
            if age < 0 or age > 1.:
                continue
            self.publisher.publish(VRInput(stamp=stamp(self.now()-age), source_id=item.source_id,
                seq=item.seq, side=item.side, runtime_mode=self.runtime_mode,
                pose=list(item.pose) if item.valid else [0.]*16,
                grip=item.grip, trigger=item.trigger, init=item.init, stop=item.stop,
                valid=item.valid, clutch_seq=item.clutch_seq, mapping=item.mapping,
                buttons=item.buttons, button_edges=list(item.button_edges), virtual_grip=item.virtual_grip))

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main():
    spin(VRInputNode, threaded=False)
