"""One bounded, isolated QP worker. Late results carry tokens and never command devices."""
import multiprocessing as mp
import math
import queue
import time
import gc


def _run(config, profile, requests, results, preload_path_checks=False):
    solver = None
    try:
        from .bimanual import BimanualQP
        solver=BimanualQP(config,profile)
        path_checks={}
        def path_check(loaded):
            if loaded not in path_checks:
                import numpy as np
                from pathlib import Path
                from .tool_geometry import make_collision_checker
                recipe=dict(config['demo'])
                recipe['left_grip_in_tool_m']=(np.linalg.inv(solver.tool) @
                    np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
                path_checks[loaded]=make_collision_checker(Path(config['urdf']).parent/recipe['display_urdf'],
                    recipe,conservative_convex=True,tool_present=loaded)
            return path_checks[loaded]
        if preload_path_checks:
            # PICO uses immutable tool/model configuration. Load both CAD
            # query contexts before ready; each request still checks its fresh
            # measured start and complete proposed path, including grip meshes.
            path_check(False);path_check(True)
        if config.get('independent_arms_enabled'):
            solver.independent_abi()
            solver.empty_collision_check=path_check(False)
        # This is the private spawned child, never the caller's interpreter.
        # Initialization has a separate startup budget. Leave GC enabled for
        # later garbage without rescanning this stable model/import graph.
        gc.collect()
        gc.freeze()
        results.put(('ready',0,None))
        while True:
            item=requests.get()
            if item is None: return
            operation,token,feedback,target,reference,dt,grasp_mode,target_stamp=item
            try:
                independent=isinstance(grasp_mode,str) and grasp_mode.startswith('independent:')
                if operation=='independent_capture' or operation=='capture' and independent:
                    value=solver.capture_independent(feedback)
                elif operation == 'capture': value=solver.capture(feedback,grasp_mode)
                elif operation == 'loaded_prepare_path':
                    value=solver.check_loaded_prepare(feedback,target)
                elif operation in ('prepare_path','gripper_path'):
                    import numpy as np
                    loaded=operation=='gripper_path'
                    check=path_check(loaded)
                    names=solver.model_names if loaded else solver.names
                    tool=np.asarray(solver.capture(feedback,'right_axial')).reshape(4,4) if loaded else np.eye(4)
                    before=np.array([feedback[n] for n in names])
                    after=np.array([target.get(n,feedback[n]) for n in names]) if loaded else np.array(target)
                    if before.shape!=after.shape or not np.isfinite(after).all():
                        raise ValueError('PREPARE_JOINT_LAYOUT')
                    for name,q in zip(names,after):
                        if not profile.limits[name][0]<=q<=profile.limits[name][1]:
                            raise ValueError('PREPARE_JOINT_LIMIT')
                    count=max(1,int(np.ceil(np.max(np.abs(after-before))/.01)))
                    samples=[]
                    for f in np.linspace(0,1,count+1):
                        q=dict(feedback);q.update(zip(names,before+f*(after-before)))
                        samples.append(dict(t=float(f),q={n:q[n] for n in solver.model_names},tool=tool))
                    value=check(samples)
                elif operation == 'solve':
                    value=(solver.solve_independent(feedback,target,active_mask=int(grasp_mode.split(':')[1]),
                             reference=reference,dt=dt) if independent else
                           solver.solve(feedback,target,reference=reference,dt=dt,target_stamp=target_stamp))
                else: raise ValueError('BIMANUAL_WORKER_OPERATION')
                diagnostic=getattr(solver,'last_geometry_step',None) if operation=='solve' and not independent else None
                if diagnostic is not None:results.put(('diagnostic',token,diagnostic))
                results.put(('result',token,value))
            except (ValueError,RuntimeError) as exc:
                results.put(('error',token,str(exc)))
    except Exception as exc:
        results.put(('fatal',0,type(exc).__name__+': '+str(exc)))
    finally:
        if solver is not None:
            solver.close()


class BimanualWorker:
    def __init__(self,config,profile,*,preload_path_checks=False):
        self.timeout=float(config.get('worker_timeout', config['solve_timeout_us']*1e-6+.2))
        self.startup_timeout=float(config.get('worker_startup_timeout',30.))
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout,self.startup_timeout)):
            raise ValueError('BIMANUAL_WORKER_TIMEOUT_CONFIG')
        ctx=mp.get_context('spawn')
        self.requests,self.results=ctx.Queue(maxsize=1),ctx.Queue(maxsize=2)
        self.process=ctx.Process(target=_run,args=(config,profile,self.requests,self.results,preload_path_checks),daemon=True)
        self.ready=self.busy=False
        self.error=''
        self.diagnostic=None
        self.submitted=0.
        self.started=time.monotonic()
        self.process.start()

    def submit(self,operation,token,feedback,target=None,*,reference=None,dt=None,grasp_mode='rigid',target_stamp=None):
        if not self.ready or self.busy: return False
        self.requests.put_nowait((operation,token,feedback,target,reference,dt,grasp_mode,target_stamp))
        self.diagnostic=None
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
            elif kind == 'diagnostic': self.diagnostic=(token,value)
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
