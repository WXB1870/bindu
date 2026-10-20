"""Shared ROS test waits and owned-process cleanup; no device IO."""
import os
import signal
import subprocess
import time
import rclpy


def wait(node, future, seconds=10):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
    if not future.done(): raise AssertionError('client future timed out')
    return future.result()


def until(node, predicate, seconds=10):
    end=time.monotonic()+seconds
    while not predicate() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
    assert predicate(), 'condition timed out'


def wait_for_task_ready(node, namespace, navigation_only=False, seconds=8.):
    from std_srvs.srv import Trigger
    client = node.create_client(Trigger, namespace+('/navigation/ready' if navigation_only else '/tasks/ready'))
    try:
        assert client.wait_for_service(timeout_sec=seconds), 'readiness service unavailable'
        deadline = time.monotonic()+seconds
        reply = wait(node,client.call_async(Trigger.Request()),seconds=seconds)
        while not reply.success and time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            reply = wait(node,client.call_async(Trigger.Request()),seconds=max(.01,deadline-time.monotonic()))
        assert reply.success, reply.message
    finally:
        node.destroy_client(client)


def terminate(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL)
            p.wait(timeout=3)
