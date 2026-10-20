"""Bounded ROS publication outside the PhysX stepping interpreter."""
import multiprocessing as mp
import queue
import time


def latest_put(items, value):
    try:
        items.put_nowait(value)
        return False
    except queue.Full:
        try:items.get_nowait()
        except queue.Empty:pass
        try:items.put_nowait(value)
        except queue.Full:pass
        return True


def publish(namespace, samples, reports, ready, stop):
    import json
    import signal
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from bindu_interfaces.msg import SimulationFeedback
    # The parent handles the process-group SIGINT and closes this publisher
    # after finishing the current step. SIGTERM retains its normal fallback.
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('snowbot_isaac_feedback',namespace=namespace)
    feedback=node.create_publisher(SimulationFeedback,'simulation/feedback',
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    health=node.create_publisher(String,'simulation/health',1)
    maxima=dict(feedback_publish_max_s=0.,health_publish_max_s=0.,queue_age_max_s=0.,
                stale_discarded=0,published=0)
    ready.set()
    try:
        while not stop.is_set():
            try:item=samples.get(timeout=.05)
            except queue.Empty:continue
            while True:
                try:item=samples.get_nowait()
                except queue.Empty:break
            message,state=item
            age=time.time()-state['stamp']
            maxima['queue_age_max_s']=max(maxima['queue_age_max_s'],age)
            # A queued state retains its physical timestamp. Old snapshots
            # are discarded rather than being restamped during DDS recovery.
            if not 0 <= age <= .2:
                maxima['stale_discarded']+=1
                latest_put(reports,dict(maxima));continue
            started=time.monotonic()
            feedback.publish(message)
            feedback_s=time.monotonic()-started
            maxima['feedback_publish_max_s']=max(maxima['feedback_publish_max_s'],feedback_s)
            maxima['published']+=1
            state=dict(state,publication=dict(state.get('publication',{}),**maxima,
                       queue_age_s=age,feedback_publish_s=feedback_s))
            started=time.monotonic()
            health.publish(String(data=json.dumps(state)))
            health_s=time.monotonic()-started
            maxima['health_publish_max_s']=max(maxima['health_publish_max_s'],health_s)
            latest_put(reports,dict(maxima))
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();rclpy.try_shutdown()


class PhysicsPublisher:
    def __init__(self, namespace, *, worker=publish):
        ctx=mp.get_context('spawn')
        self.samples=ctx.Queue(maxsize=2)
        self.reports=ctx.Queue(maxsize=2)
        self.ready=ctx.Event();self.stop=ctx.Event()
        self.latest={};self.dropped=0
        self.process=ctx.Process(target=worker,args=(namespace,self.samples,self.reports,self.ready,self.stop),daemon=True)
        self.process.start()

    def submit(self, message, state):
        if not self.process.is_alive():raise RuntimeError('PHYSICS_PUBLISHER_EXITED')
        if latest_put(self.samples,(message,state)):self.dropped+=1

    def status(self):
        while True:
            try:self.latest=self.reports.get_nowait()
            except queue.Empty:break
        return dict(self.latest,alive=self.process.is_alive(),ready=self.ready.is_set(),dropped=self.dropped)

    def close(self):
        self.stop.set();self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.terminate();self.process.join(timeout=1.)
        for items in (self.samples,self.reports):
            items.cancel_join_thread();items.close()
