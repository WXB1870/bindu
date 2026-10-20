"""DDS stalls must not block physical stepping or refresh a sampled timestamp."""
import multiprocessing as mp
import queue
import time
import unittest

from tools.simulation.snowbot_isaac_io import PhysicsPublisher, latest_put


def stalled_publisher(release, samples, reports, ready, stop):
    first=samples.get(timeout=5.)
    ready.set()
    while not release.wait(.01):
        if stop.is_set():return
    while not stop.is_set():
        try:item=samples.get(timeout=.02)
        except queue.Empty:continue
        latest_put(reports,dict(sequence=item[0],stamp=item[1]['stamp']))


class PhysicsPublicationTests(unittest.TestCase):
    def test_stalled_dds_consumer_does_not_block_the_physics_producer(self):
        release=mp.get_context('spawn').Event()
        publisher=PhysicsPublisher(release,worker=stalled_publisher)
        try:
            publisher.submit(0,dict(stamp=10.))
            self.assertTrue(publisher.ready.wait(5.))
            started=time.monotonic()
            for i in range(1000):publisher.submit(i+1,dict(stamp=10.))
            self.assertLess(time.monotonic()-started,.5)
            self.assertGreater(publisher.dropped,0)
            release.set()
            deadline=time.monotonic()+3.
            while time.monotonic()<deadline:
                publisher.submit(1001,dict(stamp=10.))
                state=publisher.status()
                if state.get('sequence')==1001:break
                time.sleep(.01)
            self.assertEqual(state['sequence'],1001)
            self.assertEqual(state['stamp'],10.)
        finally:publisher.close()
        self.assertFalse(publisher.process.is_alive())

    def test_latest_queue_is_bounded_and_keeps_the_new_snapshot(self):
        samples=queue.Queue(2)
        self.assertFalse(latest_put(samples,1))
        self.assertFalse(latest_put(samples,2))
        self.assertTrue(latest_put(samples,3))
        self.assertEqual([samples.get_nowait(),samples.get_nowait()],[2,3])

    def test_real_ros_bridge_preserves_stamp_sequence_and_joint_feedback(self):
        import os,uuid,json
        from unittest.mock import patch
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        from bindu_interfaces.msg import SimulationFeedback
        from builtin_interfaces.msg import Time
        with patch.dict(os.environ,ROS_DOMAIN_ID='224',ROS_LOCALHOST_ONLY='1'):
            rclpy.init()
            namespace='/isaac_io_'+uuid.uuid4().hex[:8]
            node=rclpy.create_node('publication_test',namespace=namespace)
            received=[];health=[]
            node.create_subscription(SimulationFeedback,'simulation/feedback',received.append,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            node.create_subscription(String,'simulation/health',lambda msg:health.append(json.loads(msg.data)),1)
            publisher=PhysicsPublisher(namespace)
            try:
                self.assertTrue(publisher.ready.wait(8.))
                end=time.monotonic()+5.
                sent={}
                while time.monotonic()<end and not (received and health):
                    sampled=time.time();ns=round(sampled*1e9);sequence=len(sent)+1
                    msg=SimulationFeedback(stamp=Time(sec=ns//1000000000,nanosec=ns%1000000000),
                        sequence=sequence,simulator_id='boot',profile_hash='profile',ready=True,code='OK')
                    msg.joints.name=['j'];msg.joints.position=[.125];msg.joints.velocity=[.25]
                    sent[sequence]=msg.stamp
                    publisher.submit(msg,dict(stamp=sampled,sequence=sequence))
                    rclpy.spin_once(node,timeout_sec=.01)
                self.assertTrue(received);self.assertTrue(health)
                self.assertEqual(received[-1].stamp,sent[received[-1].sequence])
                self.assertEqual(list(received[-1].joints.position),[.125])
                self.assertEqual(list(received[-1].joints.velocity),[.25])
                self.assertIn('publication',health[-1])
            finally:
                publisher.close();node.destroy_node();rclpy.try_shutdown()
