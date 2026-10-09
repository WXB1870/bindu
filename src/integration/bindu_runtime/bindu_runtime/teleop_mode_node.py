"""Single authority for manual teleop mode selection and safety transition."""
import math
import json
from pathlib import Path
import time
from types import SimpleNamespace
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger
from bindu_contracts.teleoperation import VRFrame, ToolPoseFrame
from bindu_teleoperation.modes import ModeSelection
from bindu_interfaces.action import SelectTeleopMode
from bindu_interfaces.msg import TeleopModeState, ExecutionState, RecorderHealth, VRInput, ToolPoseInput
from .common import RuntimeNode, seconds, stamp, spin


class TeleopModeNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop_mode')
        self.declare_parameter('initial_side', 'left')
        self.declare_parameter('supported_sides', ['left'])
        self.declare_parameter('require_single_worker', False)
        self.declare_parameter('tool_config', '')
        tool_config = self.get_parameter('tool_config').value
        tool_cfg = json.loads(Path(tool_config).read_text()) if tool_config else {}
        axial = 'right_axial_grasp' in tool_cfg
        self.declare_parameter('input_source', 'vr')
        input_source = self.get_parameter('input_source').value
        if input_source not in ('vr', 'tool_pose'):
            raise ValueError('BIMANUAL_INPUT_SOURCE')
        if input_source == 'tool_pose' and not tool_cfg.get('input_frame'):
            raise ValueError('BIMANUAL_INPUT_FRAME')
        self.tool_frame = None
        self.require_single_worker = self.get_parameter('require_single_worker').value
        self.supported_sides = tuple(self.get_parameter('supported_sides').value)
        self.selection = ModeSelection(self.profile, self.get_parameter('initial_side').value, self.now(),
                                       left_orientation=axial, tool_config=tool_cfg if input_source == 'tool_pose' else None)
        if self.selection.side not in self.supported_sides or set(self.supported_sides)-{'left','right'}:
            raise ValueError('INVALID_SUPPORTED_SIDES')
        self.state = self.health = None
        self.frames = {}
        self.paired_frames = {}
        self.busy = False
        group = ReentrantCallbackGroup()
        self.tool_client = self.create_client(Trigger, 'teleop/tool_ready', callback_group=group)
        self.single_client = self.create_client(Trigger, 'teleop/worker_ready', callback_group=group)
        self.create_subscription(ExecutionState,'execution/state',lambda m:setattr(self,'state',m),1)
        self.create_subscription(RecorderHealth,'recorder/health',self.on_health,5)
        self.create_subscription(VRInput,'teleop/vr/input',self.on_frame,5)
        self.create_subscription(ToolPoseInput,'teleop/tool/input',self.on_tool_frame,5)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(TeleopModeState,'teleop/mode_state',qos)
        self.create_timer(.05,self.publish)
        self.server = ActionServer(self,SelectTeleopMode,'teleop/select_mode',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.publish()

    def on_health(self,msg):
        if msg.run_id == self.run_id:
            self.health = msg

    def on_frame(self,msg):
        if msg.side not in ('left','right'):
            return
        self.frames[msg.side] = VRFrame(msg.source_id,msg.seq,seconds(msg.stamp),msg.side,
            tuple(msg.pose),msg.grip,msg.trigger,msg.init,msg.stop,msg.valid,msg.clutch_seq)
        left,right=self.frames.get('left'),self.frames.get('right')
        if (left and right and left.source_id==right.source_id and left.seq==right.seq and
                abs(left.stamp-right.stamp)<=.02):
            # Keep the last complete receiver event while the next side arrives.
            self.paired_frames={'left':left,'right':right}

    def on_tool_frame(self, msg):
        self.tool_frame = ToolPoseFrame(msg.source_id, msg.seq, seconds(msg.stamp),
            msg.frame_id, tuple(msg.pose), msg.tight, msg.valid, msg.stop)

    def publish(self):
        m = self.selection
        self.publisher.publish(TeleopModeState(stamp=stamp(self.now()),changed_at=stamp(m.changed_at),
            profile_hash=self.profile.digest,generation=m.generation,mode=m.mode,side=m.side,
            phase=m.phase,code=m.code))

    def goal(self,request):
        if self.busy or request.expected_generation != self.selection.generation or (
                request.mode not in ('single_arm','bimanual_tool') or
                (request.mode == 'single_arm' and request.side not in self.supported_sides) or
                (request.mode == 'bimanual_tool' and request.side) or
                not math.isfinite(request.timeout) or not .2 <= request.timeout <= 15.):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def pause(self):
        future = Future(executor=self.executor)
        timer = self.create_timer(.01, lambda:future.set_result(None) if not future.done() else None)
        try: await future
        finally: self.destroy_timer(timer)

    async def execute(self,goal):
        result = SelectTeleopMode.Result()
        try:
            changed = self.selection.begin(goal.request.mode,goal.request.side,
                                           goal.request.expected_generation,self.now())
            self.publish()
            if changed:
                end = time.monotonic()+goal.request.timeout
                reason = 'MODE_SWITCH_TIMEOUT'
                target_ready = (goal.request.mode == 'single_arm' and not self.require_single_worker)
                while time.monotonic() < end:
                    if goal.is_cancel_requested:
                        reason = 'MODE_SWITCH_CANCELED'
                        break
                    s = self.state
                    view = None if s is None else SimpleNamespace(profile_hash=s.profile_hash,
                        lease_active=s.lease_active,state=s.state,reference=s.reference,
                        stop_failures=s.stop_failures,feedback_stamp=seconds(s.feedback_stamp),joints=s.joints)
                    health = None if self.health is None else SimpleNamespace(
                        ready=self.health.ready,stamp=seconds(self.health.stamp))
                    frames = (self.paired_frames if goal.request.mode=='bimanual_tool' else self.frames)
                    reason = self.selection.ready(view,health,frames,self.now(),
                                                   tuple(self.profile.groups), tool_frame=self.tool_frame)
                    if not reason and not target_ready:
                        client = (self.tool_client if goal.request.mode == 'bimanual_tool'
                                  else self.single_client)
                        if not client.service_is_ready():
                            reason = ('BIMANUAL_NOT_READY' if goal.request.mode == 'bimanual_tool'
                                      else 'SINGLE_ARM_NOT_READY')
                        else:
                            future=client.call_async(Trigger.Request())
                            until=time.monotonic()+.5
                            while not future.done() and time.monotonic()<until:
                                await self.pause()
                            if not future.done():
                                reason='BIMANUAL_READY_TIMEOUT'
                                await self.pause()
                                continue
                            reply=future.result()
                            target_ready = reply.success
                            reason = '' if target_ready else (reply.message or 'TARGET_NOT_READY')
                    goal.publish_feedback(SelectTeleopMode.Feedback(phase='SWITCHING',code=reason))
                    if not reason:
                        self.selection.finish(self.now())
                        self.publish()
                        break
                    if (reason == 'BIMANUAL_GRASP_GEOMETRY' or
                            reason.startswith(('BIMANUAL_WORKER_FAILED',
                                               'BIMANUAL_WORKER_TIMEOUT',
                                               'BIMANUAL_WORKER_EXITED'))):
                        # A static grasp mismatch or failed solver needs a new
                        # explicit selection after the cause is corrected.
                        break
                    await self.pause()
                else:
                    reason = 'MODE_SWITCH_TIMEOUT:'+reason
                if self.selection.phase == 'SWITCHING':
                    self.selection.fail(reason,self.now())
                    self.publish()
            result.success = self.selection.phase == 'READY'
            result.code = 'OK' if result.success else self.selection.code
        except (RuntimeError, ValueError) as exc:
            self.selection.fail(str(exc),self.now())
            self.publish()
            result.code = str(exc)
        finally:
            result.generation = self.selection.generation
            self.busy = False
        if result.success: goal.succeed()
        elif goal.is_cancel_requested: goal.canceled()
        else: goal.abort()
        return result


def main():
    spin(TeleopModeNode)
