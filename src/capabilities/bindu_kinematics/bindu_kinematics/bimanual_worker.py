"""One bounded, isolated QP worker. Late results carry tokens and never command devices."""
import multiprocessing as mp
import math
import queue
import time
import gc


def prepare_limit_reference(profile, names, feedback, reference, target,
                            max_excursion, max_tracking_offset):
    """Validate an explicit inward A return from an authenticated held reference.

    The caller authenticates the PAUSE identity. Raw measured positions remain
    the geometry-validation start; all commanded references stay inside limits.
    """
    if (not all(math.isfinite(x) and x > 0 for x in (max_excursion, max_tracking_offset)) or
            set(reference) != set(names) or not set(names) <= set(feedback) or
            not set(names) <= set(target)):
        raise ValueError('PREPARE_LIMIT_RECOVERY_LAYOUT')
    result = dict(reference)
    for name in names:
        q, held, end = feedback[name], reference[name], target[name]
        lo, hi = profile.limits[name]
        outside = q < lo or q > hi
        if (not all(math.isfinite(x) for x in (q, held, end)) or
                not lo <= held <= hi or not lo <= end <= hi or
                max(lo-q, q-hi, 0.) > max_excursion or
                outside and abs(held-q) > max_tracking_offset):
            raise ValueError('PREPARE_LIMIT_RECOVERY_BOUND:'+name)
        # Only the out-of-bounds axes use the bounded-excursion contract.
        # In-bounds tracking residuals still undergo the complete CAD prefix
        # check and must not prevent another joint's explicit inward return.
        margin = profile.feedback_tolerances['joint_position']
        if (q < lo and (held < q or end < held or end < lo+margin) or
                q > hi and (held > q or end > held or end > hi-margin)):
            raise ValueError('PREPARE_LIMIT_RECOVERY_NOT_INWARD:'+name)
        result[name] = held
    return result


def prepare_limit_exit_target(profile, names, feedback, reference, target, reserve):
    """Short inward stage; other axes retain their authenticated hold targets.

    The caller still validates the measured prefix and this entire CAD path.
    The final preparation path is checked anew after measured inward arrival.
    """
    result = dict(reference)
    for name in names:
        lo, hi = profile.limits[name]
        if feedback[name] < lo:
            result[name] = min(target[name], max(reference[name], lo+reserve))
        elif feedback[name] > hi:
            result[name] = max(target[name], min(reference[name], hi-reserve))
    return result


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
                    recipe,conservative_convex=True,tool_present=loaded,validation_only=True)
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
            operation,token,feedback,target,reference,dt,grasp_mode,target_stamp,*executed=item
            solver.execution_reference=executed[0] if executed and operation=='solve' else None
            try:
                solver.last_constraint=solver.last_timing=None
                independent=isinstance(grasp_mode,str) and grasp_mode.startswith('independent:')
                if operation=='independent_capture':
                    value=solver.capture_independent(feedback,reference=reference,preserve_rigid_grasp=True)
                elif operation=='preflight_capture':
                    value=solver.capture(feedback,grasp_mode,reference=reference,commit_grasp=False)
                elif operation=='capture' and independent:
                    value=solver.capture_independent(feedback,reference=reference)
                elif operation == 'capture': value=solver.capture(feedback,grasp_mode,reference=reference)
                elif operation == 'loaded_prepare_path':
                    value=solver.check_loaded_prepare(feedback,target)
                elif operation in ('prepare_path','prepare_recovery_path','prepare_limit_recovery_path','gripper_path','gripper_body_path'):
                    import numpy as np
                    loaded=operation=='gripper_path'
                    gripper=operation in ('gripper_path','gripper_body_path')
                    check=path_check(loaded)
                    names=solver.model_names if gripper else solver.names
                    tool=np.asarray(solver.capture(feedback,'right_axial')).reshape(4,4) if loaded else np.eye(4)
                    before=np.array([feedback[n] for n in names])
                    after=np.array([target.get(n,feedback[n]) for n in names]) if gripper else np.array(target)
                    if before.shape!=after.shape or not np.isfinite(after).all():
                        raise ValueError('PREPARE_JOINT_LAYOUT')
                    if operation=='prepare_limit_recovery_path' and reference is None:
                        raise ValueError('PREPARE_LIMIT_RECOVERY_HOLD_REQUIRED')
                    for name,q in zip(names,after):
                        if not profile.limits[name][0]<=q<=profile.limits[name][1]:
                            raise ValueError('PREPARE_JOINT_LIMIT')
                    if reference is not None:
                        # A may start at a measured stop just outside a nominal
                        # limit. Validate the actual-to-legal prefix as well as
                        # the exact reference path; never hide a CAD collision.
                        start=np.array([reference[n] for n in names])
                        tolerance=profile.feedback_tolerances['joint_position']
                        limit_recovery=operation=='prepare_limit_recovery_path'
                        if limit_recovery:
                            prepare_limit_reference(profile,names,feedback,reference,
                                dict(zip(names,after)),config['prepare_limit_recovery_max_rad'],
                                config['prepare_reference_max_offset_rad'])
                        if (loaded or not np.isfinite(start).all() or
                                any(not profile.limits[n][0]<=q<=profile.limits[n][1] for n,q in zip(names,start)) or
                                not limit_recovery and np.max(np.abs(start-before))>tolerance):
                            raise ValueError('PREPARE_START_JOINT_LIMIT')
                        count=max(1,int(np.ceil(np.max(np.abs(start-before))/.001)))
                        prefix=[]
                        for f in np.linspace(0,1,count+1):
                            q=dict(feedback);q.update(zip(names,before+f*(start-before)))
                            prefix.append(dict(t=float(f),q={n:q[n] for n in solver.model_names},tool=tool))
                        check(prefix)
                        before=start
                    count=max(1,int(np.ceil(np.max(np.abs(after-before))/.01)))
                    samples=[]
                    for f in np.linspace(0,1,count+1):
                        q=dict(feedback);q.update(zip(names,before+f*(after-before)))
                        samples.append(dict(t=float(f),q={n:q[n] for n in solver.model_names},tool=tool))
                    try:
                        value=check(samples)
                    except ValueError as exc:
                        recovery=config.get('prepare_collision_recovery')
                        if (operation not in ('prepare_recovery_path','prepare_limit_recovery_path') or not recovery or
                                not str(exc).startswith('SELF_COLLISION:') or not str(exc).endswith(' t=0.0')):
                            raise
                        from .tool_geometry import InitialSelfCollisionRecovery
                        policy=InitialSelfCollisionRecovery(recovery).config
                        distance=float(np.max(np.abs(after-before)))
                        count=max(1,int(np.ceil(distance/policy['joint_sample_step_rad'])))
                        samples=[]
                        for f in np.linspace(0,1,count+1):
                            q=dict(feedback);q.update(zip(names,before+f*(after-before)))
                            samples.append(dict(t=float(f),q={n:q[n] for n in solver.model_names},tool=tool))
                        value=check(samples,initial_self_collision_recovery=policy)
                        fraction=min(1.,policy['max_exit_joint_travel_rad']/distance)
                        value['self_collision_recovery']['exit_waypoint']=dict(zip(names,before+fraction*(after-before)))
                elif operation == 'solve':
                    value=(solver.solve_independent(feedback,target,active_mask=int(grasp_mode.split(':')[1]),
                             reference=reference,dt=dt) if independent else
                           solver.solve(feedback,target,reference=reference,dt=dt,target_stamp=target_stamp))
                else: raise ValueError('BIMANUAL_WORKER_OPERATION')
                if operation=='solve':
                    diagnostic=dict(getattr(solver,'last_geometry_step',None) or {'scale':1.},
                                    timing=getattr(solver,'last_timing',None))
                    results.put(('diagnostic',token,diagnostic))
                results.put(('result',token,value))
            except (ValueError,RuntimeError) as exc:
                results.put(('diagnostic',token,dict(scale=1.,operation=operation,
                    constraint=getattr(solver,'last_constraint',None),
                    timing=getattr(solver,'last_timing',None) if operation=='solve' else None)))
                results.put(('error',token,str(exc)))
            finally:
                solver.execution_reference=None
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
        self.failure=''
        self.diagnostic=None
        self.submitted=0.
        self.started=time.monotonic()
        self.process.start()

    def submit(self,operation,token,feedback,target=None,*,reference=None,dt=None,grasp_mode='rigid',target_stamp=None,
               execution_reference=None):
        if self.failure or not self.ready or self.busy: return False
        self.requests.put_nowait((operation,token,feedback,target,reference,dt,grasp_mode,target_stamp,execution_reference))
        self.diagnostic=None
        self.busy=True
        self.submitted=time.monotonic()
        return True

    def poll(self):
        if getattr(self,'failure',''): raise RuntimeError(self.failure)
        # This is a parent receipt budget, including queueing and scheduling.
        # Draining a late result first would clear busy (or set ready) and
        # erase the very deadline it must satisfy. A faulted worker requires
        # explicit replacement; a late message cannot revive prior intent.
        now=time.monotonic()
        if ((self.busy and now-self.submitted > self.timeout) or
                (not self.ready and now-self.started > self.startup_timeout)):
            self._fail('BIMANUAL_WORKER_TIMEOUT')
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
        if self.error: self._fail('BIMANUAL_WORKER_FAILED:'+self.error)
        if not self.process.is_alive(): self._fail('BIMANUAL_WORKER_EXITED')
        return item

    def _fail(self,code):
        self.failure=code
        self.ready=False
        raise RuntimeError(code)

    def close(self):
        if self.process.is_alive(): self.process.terminate()
        self.process.join(timeout=2.)
        if self.process.is_alive(): self.process.kill();self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            q.close();q.cancel_join_thread()
