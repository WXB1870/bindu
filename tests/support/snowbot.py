"""Shared ROS test waits and owned-process cleanup; no device IO."""
import os
import signal
import subprocess
import time
import rclpy


def until(node, predicate, timeout=6.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate(): return
    raise AssertionError('ROS condition timed out')


def call(node, client, request):
    until(node, client.service_is_ready)
    future = client.call_async(request)
    until(node, future.done)
    return future.result()


def terminate(process, node=None):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        end = time.monotonic()+6.5
        while process.poll() is None and time.monotonic() < end:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=.01)
            else:
                time.sleep(.01)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2.)
