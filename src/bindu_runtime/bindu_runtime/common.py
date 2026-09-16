import uuid
import signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
from bindu_core.profile import Profile
from bindu_interfaces.msg import RuntimeEvent

EVENT_QOS = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def stamp(value):
    ns = int(value * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class RuntimeNode(Node):
    def __init__(self, name):
        super().__init__(name)
        default = str(Path(get_package_share_directory('bindu_runtime')) / 'config/wheel_sim.json')
        self.declare_parameter('profile', default)
        self.declare_parameter('run_id', 'manual')
        self.declare_parameter('simulation', True)
        if not self.get_parameter('simulation').value:
            raise RuntimeError('Only simulated adapters are implemented; real operation is refused')
        self.profile_path = self.get_parameter('profile').value
        self.profile = Profile.load(self.profile_path)
        self.run_id = self.get_parameter('run_id').value
        self.instance_id = uuid.uuid4().hex
        self.event_pub = self.create_publisher(RuntimeEvent, 'events', EVENT_QOS)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def event(self, state, code, task_id='', command_id='', observation_id=''):
        msg = RuntimeEvent(stamp=stamp(self.now()), run_id=self.run_id, task_id=task_id,
                           command_id=command_id, observation_id=observation_id,
                           component=self.get_name(), state=state, code=code, simulated=True)
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
