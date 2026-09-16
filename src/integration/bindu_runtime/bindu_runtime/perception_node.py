import time
import uuid
from rclpy.callback_groups import ReentrantCallbackGroup
from bindu_interfaces.srv import LocateObject, InjectFault
from bindu_interfaces.msg import ObjectObservation
from bindu_runtime.plugins import load_provider
from .common import RuntimeNode, stamp, spin, EVENT_QOS


class PerceptionNode(RuntimeNode):
    def __init__(self):
        super().__init__('perception')
        self.declare_parameter('perception_provider', 'bindu_perception.simulated:SimObjectLocator')
        self.locator = load_provider(self.get_parameter('perception_provider').value)
        self.delay = 0.
        self.stale = False
        self.pub = self.create_publisher(ObjectObservation, 'perception/observations', EVENT_QOS)
        self.group = ReentrantCallbackGroup()
        self.create_service(LocateObject, 'perception/locate', self.locate, callback_group=self.group)
        self.create_service(InjectFault, 'perception/sim_fault', self.fault, callback_group=self.group)

    def fault(self, request, reply):
        reply.ok = request.fault in ('', 'delay', 'stale') and 0 <= request.value <= 3
        if reply.ok:
            self.delay = request.value if request.fault == 'delay' else 0.
            self.stale = request.fault == 'stale'
        reply.code = 'OK' if reply.ok else 'UNKNOWN_FAULT'
        return reply

    def locate(self, request, reply):
        captured = self.now()
        # Bounded synthetic delay lives in a worker callback, never in execution.
        time.sleep(self.delay)
        reply.request_id = request.request_id
        reply.object_id = request.object_id
        reply.simulated = True
        estimate = self.locator.locate(request.object_id)
        reply.found = request.schema_version == 1 and estimate.found
        reply.code = 'OK' if reply.found else 'UNSUPPORTED_OBJECT_OR_SCHEMA'
        reply.observation_id = uuid.uuid4().hex
        reply.pose.header.frame_id = estimate.frame
        reply.pose.header.stamp = stamp(captured - (2. if self.stale else 0.))
        reply.pose.pose.position.x, reply.pose.pose.position.y, reply.pose.pose.position.z = estimate.position
        reply.pose.pose.orientation.w = 1.
        self.pub.publish(ObjectObservation(request_id=request.request_id, task_id=request.task_id,
                         observation_id=reply.observation_id, object_id=reply.object_id,
                         pose=reply.pose, found=reply.found, simulated=True))
        self.event('OBSERVATION', reply.code, request.task_id, observation_id=reply.observation_id)
        return reply


def main():
    spin(PerceptionNode)
