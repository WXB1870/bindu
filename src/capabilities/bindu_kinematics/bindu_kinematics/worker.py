"""Bounded process isolation. One in-flight solve; caller retains newest input."""
import multiprocessing as mp
import queue
import time


def _run(config, requests, results):
    try:
        from .pinocchio_casadi import PinocchioCasadiIK
        solver = PinocchioCasadiIK(config)
        results.put(('ready', None))
        while True:
            request = requests.get()
            if request is None:
                return
            results.put(('result', solver.solve(request)))
    except Exception as exc:
        results.put(('error', type(exc).__name__ + ': ' + str(exc)))


class KinematicsWorker:
    def __init__(self, config):
        ctx = mp.get_context('spawn')
        self.requests, self.results = ctx.Queue(maxsize=1), ctx.Queue(maxsize=2)
        self.process = ctx.Process(target=_run, args=(config, self.requests, self.results), daemon=True)
        self.ready, self.busy = False, False
        self.started = time.monotonic()
        self.submitted = 0.
        self.timeout = config['solve_timeout'] + .2
        self.process.start()

    def submit(self, request):
        if not self.ready or self.busy:
            return False
        self.requests.put_nowait(request)
        self.busy, self.submitted = True, time.monotonic()
        return True

    def poll(self):
        result = None
        for _ in range(2):
            try:
                kind, item = self.results.get_nowait()
            except queue.Empty:
                break
            if kind == 'error':
                raise RuntimeError('IK_WORKER_FAILED: '+item)
            if kind == 'ready':
                self.ready = True
            else:
                self.busy, result = False, item
        if not self.process.is_alive():
            raise RuntimeError('IK_WORKER_EXITED')
        if (self.busy and time.monotonic()-self.submitted > self.timeout) or (
                not self.ready and time.monotonic()-self.started > 30.):
            raise RuntimeError('IK_WORKER_TIMEOUT')
        return result

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1.)
        for q in (self.requests, self.results):
            q.close()
            q.cancel_join_thread()
