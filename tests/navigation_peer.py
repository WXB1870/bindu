#!/usr/bin/env python3
"""Action/velocity producer fixture. No path planner, SLAM or obstacle model."""
import math
import time
import uuid
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from bindu_interfaces.msg import ExecutionState, NavigationPose, NavigationVelocity
from bindu_runtime.common import RuntimeNode, seconds, stamp, spin


class Peer(RuntimeNode):
    def __init__(self):
        super().__init__('navigation_peer')
        self.declare_parameter('case', 'normal')
        self.case = self.get_parameter('case').value
        self.group = ReentrantCallbackGroup()
        self.state = None
        self.active = None
        self.previous = None
        self.sequence = 0
        self.started = 0.
        self.source = uuid.uuid4().hex
        self.create_subscription(ExecutionState, 'execution/state', lambda m:setattr(self,'state',m), 1, callback_group=self.group)
        self.velocity_pub = self.create_publisher(NavigationVelocity, 'navigation/backend/velocity', QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pose_pub = self.create_publisher(NavigationPose, 'navigation/backend/pose', QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.03,self.tick,callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, 'navigation/backend/navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.goal, cancel_callback=lambda _:CancelResponse.ACCEPT,
            callback_group=self.group)

    def goal(self, request):
        if self.case == 'late_accept':
            time.sleep(2.4)
        return GoalResponse.REJECT if self.case=='reject' else GoalResponse.ACCEPT

    async def pause(self):
        f=Future(executor=self.executor)
        t=self.create_timer(.01,lambda:f.set_result(None) if not f.done() else None,callback_group=self.group)
        try: await f
        finally: self.destroy_timer(t)

    def tick(self):
        if self.state is None:
            return
        elapsed=time.monotonic()-self.started
        if not (self.active and self.case=='pose_loss' and elapsed>.25):
            p=NavigationPose(map_id='navigation_fixture_v1')
            p.pose.header.frame_id='map'
            p.pose.header.stamp=self.state.feedback_stamp
            p.pose.pose.position.x=self.state.base_x
            p.pose.pose.position.y=self.state.base_y
            p.pose.pose.orientation.z=math.sin(self.state.base_yaw/2)
            p.pose.pose.orientation.w=math.cos(self.state.base_yaw/2)
            if self.active and self.case=='wrong_map': p.map_id='wrong_version'
            self.pose_pub.publish(p)
        if self.active is None:
            return
        if self.case=='input_loss' and elapsed>.25:
            return
        goal_id=bytes(self.active.goal_id.uuid).hex()
        error=self.active.request.pose.pose.position.x-self.state.base_x
        v=max(-.15,min(.15,error*2))
        w=0.
        if abs(error)<.015: v=0.
        if self.case == 'differential_turn':
            target=self.active.request.pose.pose
            dx,dy=target.position.x-self.state.base_x,target.position.y-self.state.base_y
            distance=math.hypot(dx,dy)
            heading=math.atan2(dy,dx)-self.state.base_yaw
            heading=math.atan2(math.sin(heading),math.cos(heading))
            direction=1.
            if abs(heading)>math.pi/2:
                direction=-1.
                heading=math.atan2(math.sin(heading+math.pi),math.cos(heading+math.pi))
            if distance<.015:
                v=0.
                yaw=2*math.atan2(target.orientation.z,target.orientation.w)
                heading=math.atan2(math.sin(yaw-self.state.base_yaw),math.cos(yaw-self.state.base_yaw))
            else:
                v=direction*min(.15,distance*2) if abs(heading)<.3 else 0.
            w=max(-.4,min(.4,heading*2)) if abs(heading)>.015 else 0.
        if self.case=='timeout': v=0.
        if self.case=='source_change' and elapsed>.25: self.source='restarted'
        self.sequence+=1
        msg=NavigationVelocity(goal_id=goal_id,source_id=self.source,sequence=self.sequence)
        msg.command.header.stamp=stamp(self.now())
        msg.command.header.frame_id='base_link'
        msg.command.twist.linear.x=v
        msg.command.twist.angular.z=w
        if self.case=='invalid_axis': msg.command.twist.linear.y=.1
        self.velocity_pub.publish(msg)
        if self.previous and self.case=='late_old_goal':
            # A fresh but high-sequence velocity for the previous UUID is unsafe
            # if the consumer naively assigns the current goal at receipt time.
            old=NavigationVelocity(goal_id=self.previous,source_id=self.source,sequence=100000+self.sequence)
            old.command.header.stamp=stamp(self.now())
            old.command.header.frame_id='base_link'
            old.command.twist.linear.x=.39
            self.velocity_pub.publish(old)

    async def execute(self, goal):
        self.active=goal
        self.started=time.monotonic()
        self.sequence=0
        result=NavigateToPose.Result()
        try:
            while rclpy.ok():
                elapsed=time.monotonic()-self.started
                if goal.is_cancel_requested:
                    goal.canceled()
                    return result
                if self.case=='abort' and elapsed>.3:
                    result.error_code=1; result.error_msg='fixture_failure'; goal.abort(); return result
                if self.case in ('false_success','false_yaw_success') and elapsed>.05:
                    goal.succeed(); return result
                if self.case not in ('timeout','input_loss','pose_loss','source_change','invalid_axis','wrong_map','cancel','backend_loss','feedback_loss') and self.state:
                    target=goal.request.pose.pose
                    error=math.hypot(self.state.base_x-target.position.x,self.state.base_y-target.position.y)
                    yaw=2*math.atan2(target.orientation.z,target.orientation.w)-self.state.base_yaw
                    yaw=abs(math.atan2(math.sin(yaw),math.cos(yaw)))
                    if elapsed>.3 and error<.025 and yaw<.03 and abs(self.state.base_velocity.linear.x)<.02 and abs(self.state.base_velocity.angular.z)<.02:
                        goal.succeed(); return result
                if elapsed>15:
                    result.error_code=1; goal.abort(); return result
                await self.pause()
        finally:
            self.previous=bytes(goal.goal_id.uuid).hex()
            self.active=None
        return result


if __name__=='__main__': spin(Peer, threaded=False)
