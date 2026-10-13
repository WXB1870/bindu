import uuid
import signal
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
from bindu_contracts.profile import Profile
from bindu_contracts.runtime_mode import resolve_mode, message_matches, SIMULATION, REAL
from bindu_interfaces.msg import RuntimeEvent

EVENT_QOS = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
# The mode authority publishes every 50 ms. Its heartbeat may cross a few
# scheduling periods without changing authority; joint feedback has its own
# tighter freshness check in each session.
MODE_STATE_MAX_AGE = .5


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def stamp(value):
    ns = int(value * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def joints_at_rest(joints, names, tolerance=.01):
    """Require complete, finite velocity feedback for the named joints."""
    if len(joints.name) != len(joints.velocity) or len(set(joints.name)) != len(joints.name):
        return False
    velocities = dict(zip(joints.name, joints.velocity))
    return bool(names) and all(n in velocities and math.isfinite(velocities[n]) and
                               abs(velocities[n]) <= tolerance for n in names)


class RuntimeNode(Node):
    def __init__(self, name, *, allow_hardware=False, default_mode=None):
        super().__init__(name)
        default = str(Path(get_package_share_directory('bindu_runtime')) / 'config/wheel_sim.json')
        self.declare_parameter('profile', default, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('run_id', 'manual', descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('runtime_mode', '')
        self.declare_parameter('simulation', None, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.runtime_mode = resolve_mode(self.get_parameter('runtime_mode').value,
            self.get_parameter('simulation').value,
            default=default_mode or (REAL if allow_hardware else SIMULATION))
        self.simulation = self.runtime_mode == SIMULATION
        self.set_parameters([Parameter('runtime_mode', value=self.runtime_mode),
                             Parameter('simulation', value=self.simulation)])
        for key in ('runtime_mode', 'simulation'):
            self.set_descriptor(key, ParameterDescriptor(read_only=True,
                type=self.get_parameter(key).type_.value,
                description='Robot environment selected at startup; restart to change mode'))
        if not self.simulation and not allow_hardware:
            raise RuntimeError('RUNTIME_REAL_BACKEND_NOT_IMPLEMENTED:'+name)
        self.profile_path = self.get_parameter('profile').value
        self.profile = Profile.load(self.profile_path)
        self.declare_parameter('robot_config', '', descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('robot_config_digest', '', descriptor=ParameterDescriptor(read_only=True))
        robot_config = self.get_parameter('robot_config').value
        if robot_config:
            from .robot_setup import RobotSetup
            robot = RobotSetup.load(robot_config)
            if (robot.profile.digest != self.profile.digest or
                    robot.digest != self.get_parameter('robot_config_digest').value):
                raise RuntimeError('ROBOT_CONFIG_CHANGED_SINCE_LAUNCH')
        self.run_id = self.get_parameter('run_id').value
        self.instance_id = uuid.uuid4().hex
        self.event_pub = self.create_publisher(RuntimeEvent, 'events', EVENT_QOS)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def matches_mode(self, message):
        return message_matches(message, self.runtime_mode)

    def store_mode_message(self, attribute, message):
        setattr(self, attribute, message if self.matches_mode(message) else None)

    def event(self, state, code, task_id='', command_id='', observation_id=''):
        msg = RuntimeEvent(stamp=stamp(self.now()), run_id=self.run_id, task_id=task_id,
                           command_id=command_id, observation_id=observation_id,
                           component=self.get_name(), state=state, code=code, simulated=self.simulation,
                           runtime_mode=self.runtime_mode)
        self.event_pub.publish(msg)


def spin(factory, threaded=True):
    rclpy.init()
    node = factory()
    executor = MultiThreadedExecutor(num_threads=6) if threaded else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Terminal SIGINT and launch's forwarded SIGINT may arrive together.
        # Once teardown starts, let the bounded writer flush finish once.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
