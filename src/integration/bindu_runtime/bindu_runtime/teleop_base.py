"""Optional PICO-owned cmd_vel adapter, separate from joint position execution."""
import json
import time
from pathlib import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from bindu_teleoperation.base import BaseTeleop
from .common import seconds


class TeleopBase:
    def __init__(self, node, config_path, command_enabled):
        self.node = node
        self.core = BaseTeleop(json.loads(Path(config_path).read_text()))
        self.cfg = self.core.cfg
        # Keep the default receiver-watchdog requirement. A site can explicitly
        # defer it; a missing timeout stays unknown rather than being fabricated.
        self.writable = bool(command_enabled and not node.simulation and
            (self.cfg['driver_watchdog_s'] is not None or not self.cfg['require_driver_watchdog']))
        self.disabled_reason = ('BASE_SIMULATION_PREVIEW' if node.simulation else
            'BASE_COMMANDS_DISABLED' if not command_enabled else 'BASE_DRIVER_WATCHDOG_UNCONFIRMED')
        self.publisher = None
        self.last_published = None
        self.feedback_gid = None
        self.feedback_source_fault = False
        self.graph_ok = False
        self.graph_checked_at = float('-inf')
        self.graph_reason = 'BASE_ENDPOINTS_UNAVAILABLE'
        if self.writable:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = node.create_publisher(Twist, self.cfg['cmd_vel_topic'], qos)
            if self.cfg['require_feedback']:
                node.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.core.inhibit(self.disabled_reason, node.now())

    def ingest(self, pair, now):
        self.core.ingest(*pair, now, time.monotonic())

    def on_odom(self, msg):
        if not self.cfg['require_feedback']:
            return
        now, clock = self.node.now(), time.monotonic()
        velocity = msg.twist.twist
        if not self.graph_ok or msg.child_frame_id != self.cfg['feedback_frame']:
            self.core.feedback_velocity = None
            self.core.stationary_since = None
            return
        self.core.observe(seconds(msg.header.stamp), (velocity.linear.x, velocity.angular.z), now, clock)

    def check_graph(self, clock):
        if clock-self.graph_checked_at < .1:
            return
        self.graph_checked_at = clock
        if self.feedback_source_fault:
            self.graph_ok = False
            self.core.feedback_velocity = None
            self.core.stationary_since = None
            self.graph_reason = 'BASE_FEEDBACK_SOURCE_CHANGED_RESTART_REQUIRED'
            return
        pubs = self.node.get_publishers_info_by_topic(self.cfg['cmd_vel_topic'])
        own = lambda info: (info.node_name == self.node.get_name() and
                           info.node_namespace == self.node.get_namespace())
        receivers = self.node.get_subscriptions_info_by_topic(self.cfg['cmd_vel_topic'])
        def named(info, key):
            return (not self.cfg[key] or
                info.node_namespace.rstrip('/')+'/'+info.node_name == self.cfg[key])
        self.graph_ok = (len(pubs) == 1 and own(pubs[0]) and bool(receivers) and
            all(info.topic_type == 'geometry_msgs/msg/Twist' for info in receivers) and
            any(named(info, 'receiver_node') for info in receivers))
        if self.graph_ok and self.cfg['require_feedback']:
            feedback = self.node.get_publishers_info_by_topic(self.cfg['odom_topic'])
            self.graph_ok = (len(feedback) == 1 and
                feedback[0].topic_type == 'nav_msgs/msg/Odometry' and named(feedback[0], 'feedback_node'))
        if not self.graph_ok:
            self.graph_reason = 'BASE_ENDPOINTS_UNAVAILABLE_OR_MULTIPLE_WRITERS'
        elif self.cfg['require_feedback']:
            raw_gid = getattr(feedback[0], 'endpoint_gid', None)
            try:
                gid = bytes(raw_gid) if raw_gid is not None else b''
            except (TypeError, ValueError):
                gid = b''
            if not any(gid):
                self.graph_ok = False
                self.graph_reason = 'BASE_FEEDBACK_IDENTITY_UNAVAILABLE'
            elif self.feedback_gid is None:
                self.feedback_gid = gid
            elif gid != self.feedback_gid:
                self.graph_ok = False
                self.feedback_source_fault = True
                self.graph_reason = 'BASE_FEEDBACK_SOURCE_CHANGED_RESTART_REQUIRED'
        if not self.graph_ok:
            self.core.feedback_velocity = None
            self.core.stationary_since = None

    def tick(self, control, healthy):
        now, clock = self.node.now(), time.monotonic()
        if self.writable:
            self.check_graph(clock)
        allowed = (self.writable and self.graph_ok and healthy and not control.pending and
                   self.core.source == control.source and self.core.seq == control.seq and
                   control.phase in ('standby', 'tool_standby', 'following'))
        reason = (self.disabled_reason if not self.writable else self.graph_reason if not self.graph_ok
                  else 'BASE_PICO_NOT_PERMITTED')
        velocity = self.core.tick(now, clock, allowed=allowed, reason=reason)
        if self.publisher:
            msg = Twist()
            msg.linear.x, msg.angular.z = velocity
            # Repeat zero while inhibited. Process death still needs the device
            # watchdog: no ROS publisher can stop a latched receiver after death.
            self.publisher.publish(msg)
            if velocity == (0., 0.) and self.last_published != (0., 0.):
                self.core.zero_requested = self.node.now()
                self.core.stationary_since = None
            self.last_published = velocity

    def snapshot(self):
        now, clock = self.node.now(), time.monotonic()
        fresh = (0 <= now-self.core.input_stamp <= self.cfg['input_max_age_s'] and
                 0 <= clock-self.core.received <= self.cfg['input_max_age_s'])
        return dict(writable=self.writable, reason=self.core.reason, armed=self.core.armed,
            cmd_vel_topic=self.cfg['cmd_vel_topic'], frame=self.cfg['feedback_frame'],
            receiver_node=self.cfg['receiver_node'], feedback_node=self.cfg['feedback_node'],
            driver_watchdog_s=self.cfg['driver_watchdog_s'],
            require_driver_watchdog=self.cfg['require_driver_watchdog'],
            require_feedback=self.cfg['require_feedback'],
            requested=list(self.core.target) if fresh else None, command=list(self.core.command),
            measured=list(self.core.feedback_velocity) if self.core.healthy(now, clock) else None,
            stop_confirmed=(self.core.stopped(now, clock)
                            if self.writable and self.cfg['require_feedback'] else None),
            source_id=self.core.source, seq=self.core.seq,
            input_stamp=self.core.input_stamp if self.core.source else None)

    def close(self):
        if self.publisher:
            self.core.inhibit('BASE_SHUTDOWN_STOP_REQUESTED', self.node.now())
            self.publisher.publish(Twist())
