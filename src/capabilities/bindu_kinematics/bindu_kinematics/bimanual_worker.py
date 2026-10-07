"""One bounded, isolated QP worker. Late results carry tokens and never command devices."""
import multiprocessing as mp
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
            operation,token,feedback,target=item
            try:
                if operation == 'capture': value=solver.capture(feedback)
                else: value=solver.solve(feedback,target)
                results.put(('result',token,value))
            except (ValueError,RuntimeError) as exc:
                results.put(('error',token,str(exc)))
    except Exception as exc:
        results.put(('fatal',0,type(exc).__name__+': '+str(exc)))


class BimanualWorker:
    def __init__(self,config,profile):
        ctx=mp.get_context('spawn')
        self.requests,self.results=ctx.Queue(maxsize=1),ctx.Queue(maxsize=2)
        self.process=ctx.Process(target=_run,args=(config,profile,self.requests,self.results),daemon=True)
        self.ready=self.busy=False
        self.error=''
        self.submitted=0.
        self.process.start()

    def submit(self,operation,token,feedback,target=None):
        if not self.ready or self.busy: return False
        self.requests.put_nowait((operation,token,feedback,target))
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
        if self.busy and time.monotonic()-self.submitted > .25:
            raise RuntimeError('BIMANUAL_WORKER_TIMEOUT')
        return item

    def close(self):
        if self.process.is_alive(): self.process.terminate()
        self.process.join(timeout=2.)
        if self.process.is_alive(): self.process.kill();self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            q.close();q.cancel_join_thread()
