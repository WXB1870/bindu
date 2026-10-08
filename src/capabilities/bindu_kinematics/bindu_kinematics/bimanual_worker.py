"""One bounded, isolated QP worker. Late results carry tokens and never command devices."""
import multiprocessing as mp
import math
import queue
import time


def _run(config, profile, requests, results):
    try:
        from .bimanual import BimanualQP
        solver=BimanualQP(config,profile)
        results.put(('ready',0,None))
        while True:
            item=requests.get()
            if item is None: return
            operation,token,feedback,target,reference,dt=item
            try:
                if operation == 'capture': value=solver.capture(feedback)
                elif operation == 'solve':
                    value=solver.solve(feedback,target,reference=reference,dt=dt)
                else: raise ValueError('BIMANUAL_WORKER_OPERATION')
                results.put(('result',token,value))
            except (ValueError,RuntimeError) as exc:
                results.put(('error',token,str(exc)))
    except Exception as exc:
        results.put(('fatal',0,type(exc).__name__+': '+str(exc)))


class BimanualWorker:
    def __init__(self,config,profile):
        self.timeout=float(config.get('worker_timeout', config['solve_timeout_us']*1e-6+.2))
        self.startup_timeout=float(config.get('worker_startup_timeout',30.))
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout,self.startup_timeout)):
            raise ValueError('BIMANUAL_WORKER_TIMEOUT_CONFIG')
        ctx=mp.get_context('spawn')
        self.requests,self.results=ctx.Queue(maxsize=1),ctx.Queue(maxsize=2)
        self.process=ctx.Process(target=_run,args=(config,profile,self.requests,self.results),daemon=True)
        self.ready=self.busy=False
        self.error=''
        self.submitted=0.
        self.started=time.monotonic()
        self.process.start()

    def submit(self,operation,token,feedback,target=None,*,reference=None,dt=None):
        if not self.ready or self.busy: return False
        self.requests.put_nowait((operation,token,feedback,target,reference,dt))
        self.busy=True
        self.submitted=time.monotonic()
        return True

    def poll(self):
        item=None
        for _ in range(2):
            try: kind,token,value=self.results.get_nowait()
            except queue.Empty: break
            if kind == 'ready': self.ready=True
            elif kind == 'fatal': self.error=value
            else:
                self.busy=False
                item=(kind,token,value)
        if self.error: raise RuntimeError('BIMANUAL_WORKER_FAILED:'+self.error)
        if not self.process.is_alive(): raise RuntimeError('BIMANUAL_WORKER_EXITED')
        if ((self.busy and time.monotonic()-self.submitted > self.timeout) or
                (not self.ready and time.monotonic()-self.started > self.startup_timeout)):
            raise RuntimeError('BIMANUAL_WORKER_TIMEOUT')
        return item

    def close(self):
        if self.process.is_alive(): self.process.terminate()
        self.process.join(timeout=2.)
        if self.process.is_alive(): self.process.kill();self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            q.close();q.cancel_join_thread()
