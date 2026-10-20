"""Simulation-only IPC boundary to the production QP/session/executor.

Isaac imports only ControlClient. The isolated system-Python process imports
ROS Pinocchio/native QP libraries; no ROS nodes or hardware drivers are started.
Only measured joints/velocities, local F/T wrenches and operator packets reach it.
Tool ground truth stays in the simulator for independent scoring.
"""
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time

from tools import ROOT
LOADED_SOURCE = Path(__file__).read_bytes()


class ControlClient:
    def __init__(self, config, output, feedback=None):
        self.output = Path(output)
        (self.output/'controller_loaded.py').write_bytes(LOADED_SOURCE)
        self.stderr = (self.output/'controller.stderr.log').open('w')
        packages = ('shared/bindu_contracts', 'control/bindu_execution',
                    'hardware/bindu_hardware', 'capabilities/bindu_kinematics',
                    'capabilities/bindu_teleoperation')
        env = os.environ.copy()
        for key in ('PYTHONHOME','PYTHONEXE','LD_PRELOAD','CONDA_PREFIX','VIRTUAL_ENV'):
            env.pop(key,None)
        env['PYTHONPATH'] = os.pathsep.join([str(ROOT)]+[str(ROOT/'src'/p) for p in packages]+[
            '/opt/ros/jazzy/lib/python3.12/site-packages',
            '/opt/ros/jazzy/local/lib/python3.12/dist-packages'])
        env['AMENT_PREFIX_PATH'] = os.pathsep.join([str(p) for p in (ROOT/'install').iterdir()
            if (p/'share/ament_index').exists()]+['/opt/ros/jazzy'])
        env['LD_LIBRARY_PATH'] = '/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu'
        env['OPENBLAS_NUM_THREADS'] = env['OMP_NUM_THREADS'] = '1'
        feedback_path = self.output/'controller_feedback_config.json'
        feedback_path.write_text(json.dumps(feedback)+'\n')
        self.proc = subprocess.Popen(['/usr/bin/python3','-u',str(Path(__file__).resolve()),
            '--worker',str(config),str(self.output),str(feedback_path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=self.stderr,text=True,bufsize=1,env=env)
        if not select.select([self.proc.stdout],[],[],30.)[0]:
            self.close(); raise RuntimeError('PHYSICS_CONTROL_START_TIMEOUT')
        hello = json.loads(self.proc.stdout.readline())
        if not hello.get('ready'):
            self.close(); raise RuntimeError('PHYSICS_CONTROL_START:'+str(hello))
        self.hello = hello
        self.pending = None
        self.seq = 0
        self.records = []
        self.buffer = b''

    def send(self, stamp, positions, velocities, frame, *, cancel=False, freeze_feedback=False, wrenches=None):
        if self.pending is not None:
            return False
        self.seq += 1
        sent_at = time.monotonic()
        packet = dict(seq=self.seq, stamp=stamp, q=positions, qd=velocities, frame=frame,
                      cancel=cancel, freeze_feedback=freeze_feedback, wrenches=wrenches,sent_at_wall=sent_at)
        self.proc.stdin.write(json.dumps(packet)+'\n'); self.proc.stdin.flush()
        self.pending = (self.seq,sent_at,stamp)
        return True

    def poll(self, now):
        if self.proc.poll() is not None:
            raise RuntimeError('PHYSICS_CONTROL_EXITED')
        if not select.select([self.proc.stdout],[],[],0)[0]:
            if self.pending and time.monotonic()-self.pending[1] > .23:
                raise RuntimeError('PHYSICS_CONTROL_REPLY_TIMEOUT')
            return None
        data = os.read(self.proc.stdout.fileno(),65536)
        if not data:
            raise RuntimeError('PHYSICS_CONTROL_EOF')
        self.buffer += data
        if b'\n' not in self.buffer:
            return None
        raw,self.buffer = self.buffer.split(b'\n',1)
        response = json.loads(raw)
        if self.pending is None or response.get('seq') != self.pending[0]:
            raise RuntimeError('PHYSICS_CONTROL_SEQUENCE')
        response['applied_at_sim_s'] = now
        response['ipc_wall_s'] = time.monotonic()-self.pending[1]
        response['worker_wall_s'] = response['completed_at_wall']-self.pending[1]
        if response['worker_wall_s'] > .23:
            raise RuntimeError('PHYSICS_CONTROL_REPLY_TIMEOUT')
        self.pending = None
        self.records.append(response)
        if response.get('fatal'):
            raise RuntimeError('PHYSICS_CONTROL_FATAL:'+response['fatal'])
        return response

    def close(self):
        proc = getattr(self,'proc',None)
        if proc is not None and proc.poll() is None:
            try:
                proc.stdin.close()
                proc.wait(timeout=3.)
            except (OSError,subprocess.TimeoutExpired):
                proc.terminate()
                try: proc.wait(timeout=2.)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
        if hasattr(self,'records'):
            (self.output/'controller_responses.json').write_text(json.dumps(self.records)+'\n')
        self.stderr.close()


class GripLoadObserver:
    """Calibrated Kelvin-Voigt inversion, not an additional admittance loop.

    Input is external tool-on-hand wrench at the left F/T origin in its local
    frame. Hand mass/tare must already be removed. Sensor-to-grip moment shift
    uses the known grasp offset. Isotropic stiffness/damping are assumptions in
    this experiment; real deployment requires measured calibration and bounds.
    """
    def __init__(self, parameters, config):
        import numpy as np
        self.np = np
        self.p = parameters
        self.frame = config['left_frame']
        self.offset = np.array(config['demo']['grip_offset_m'])
        self.mount = np.array(config['left_to_tool']).reshape(4,4)
        self.tool_grip = (np.linalg.inv(self.mount) @ np.r_[self.offset,1.])[:3]
        self.last_stamp = None
        self.translation = self.rotation = None
        for key in ('grip_translation_stiffness_n_m','grip_translation_damping_ns_m',
                    'grip_rotation_stiffness_nm_rad','grip_rotation_damping_nm_s_rad','max_age_s'):
            if not np.isfinite(parameters[key]) or parameters[key] <= 0:
                raise ValueError('GRIP_LOAD_CALIBRATION')

    def estimate(self, hand, sample, now):
        from scipy.spatial.transform import Rotation
        np = self.np
        if (sample is None or sample['frame_id'] != self.frame or
                not 0 <= now-sample['stamp'] <= self.p['max_age_s']):
            raise ValueError('GRIP_LOAD_STALE_OR_FRAME')
        wrench = np.asarray(sample['wrench'],dtype=float)
        if wrench.shape != (6,) or not np.isfinite(wrench).all():
            raise ValueError('GRIP_LOAD_INVALID')
        stamp = sample['stamp']
        if self.last_stamp is not None and stamp < self.last_stamp:
            raise ValueError('GRIP_LOAD_UNORDERED')
        force = -hand[:3,:3] @ wrench[:3]
        moment = -hand[:3,:3] @ (wrench[3:]-np.cross(self.offset,wrench[:3]))
        def integrate(previous,value,k,c):
            if previous is None: return value/k
            decay = np.exp(-k*(stamp-self.last_stamp)/c)
            return decay*previous+(1-decay)*value/k
        self.translation = integrate(self.translation,force,self.p['grip_translation_stiffness_n_m'],self.p['grip_translation_damping_ns_m'])
        self.rotation = integrate(self.rotation,moment,self.p['grip_rotation_stiffness_nm_rad'],self.p['grip_rotation_damping_nm_s_rad'])
        self.last_stamp = stamp
        result = np.eye(4)
        result[:3,:3] = Rotation.from_rotvec(-self.rotation).as_matrix() @ hand[:3,:3] @ self.mount[:3,:3]
        result[:3,3] = hand[:3,3]+hand[:3,:3] @ self.offset-self.translation-result[:3,:3] @ self.tool_grip
        return result


class PhysicsController:
    def __init__(self, config_path, output, feedback=None):
        import gc
        import bindu_execution
        bindu_execution.__path__.append(str(ROOT/'install/bindu_execution/lib/python3.12/site-packages/bindu_execution'))
        from bindu_execution.executor import Executor
        from bindu_teleoperation.bimanual_session import ToolPoseSession, AcceptedReference
        from bindu_hardware.robot_io.system import RobotIO
        from bindu_contracts.profile import Profile
        from bindu_kinematics.bimanual import BimanualQP
        self.config_path = Path(config_path)
        self.cfg = json.loads(self.config_path.read_text())
        self.profile = Profile.load(self.config_path.parent/self.cfg['demo']['profile'])
        self.cfg['urdf'] = str(ROOT/'src/hardware/bindu_description/urdf'/self.cfg['urdf'])
        self.observer = None
        if feedback is not None:
            if feedback.get('simulation_only') is not True: raise ValueError('SIMULATION_FEEDBACK_REQUIRED')
            self.cfg.update(max_tool_feedback_translation_m=feedback['max_translation_m'],
                            max_tool_feedback_rotation_rad=feedback['max_rotation_rad'])
            self.observer = GripLoadObserver(feedback,self.cfg)
        self.solver = BimanualQP(self.cfg,self.profile)
        self.output = Path(output)
        self.now = 0.
        self.feedback_stamp = None
        self.seq = 0
        self.initialized = False
        self.failure = ''
        self.closing = False
        self.stopped = False
        self.capture_mode = 'rigid'
        self.last_generation = 0
        self.next_renewal = 0.
        self.accepted = 0
        self.events = []
        self.hold_requested = False
        self.lease = None
        self.startup_at = None
        self.targets = dict(self.cfg['demo']['initial_positions_rad'])
        gc.collect();gc.freeze()

    def initialize(self, packet):
        from bindu_contracts.devices import JointReading, BaseReading
        from bindu_hardware.robot_io.system import RobotIO
        from bindu_execution.executor import Executor
        from bindu_teleoperation.bimanual_session import ToolPoseSession, AcceptedReference
        outer = self
        class Joints:
            def __init__(self,names): self.names = names
            def read(self,now,dt):
                q = {n:outer.measured[n] for n in self.names}
                qd = {n:outer.velocities[n] for n in self.names if n in outer.velocities}
                return JointReading(outer.feedback_stamp,q,True,qd)
            def write_positions(self,target): outer.targets.update(target)
            def request_stop(self):
                # Retain the last accepted resting reference and its preload.
                # Stop confirmation still requires new measured zero velocity.
                outer.hold_requested = True
        class Base:
            def read(self,now,dt): return BaseReading(outer.feedback_stamp,0.,0.,(0.,0.),True,y=0.)
            def write_velocity(self,velocity):
                if tuple(velocity) != (0.,0.): raise RuntimeError('FIXED_BASE_COMMAND')
            def request_stop(self): pass
        self.io = RobotIO(self.profile,{g:Joints(names) for g,names in self.profile.groups.items()},Base())
        # This numerical experiment uses physical simulation time for both
        # execution budgets. IPC/native-solve wall deadlines remain independent.
        self.executor = Executor(self.profile,self.io,steady=lambda:self.now)
        self.session = ToolPoseSession(self.cfg,0,-1.)
        self.reference = AcceptedReference(self.cfg.get('max_integration_interval',.1))
        modules = ('bindu_kinematics.bimanual','bindu_execution.executor','bindu_execution.interpolation',
                   'bindu_execution._native','bindu_teleoperation.bimanual_session','bindu_hardware.robot_io.system')
        paths = [Path(sys.modules[m].__file__).resolve() for m in modules]
        paths += [Path(self.solver.lib._name), self.config_path, Path(self.cfg['urdf'])]
        (self.output/'controller_sources.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in paths},indent=2)+'\n')
        self.initialized = True

    def stop(self, code):
        if self.closing:
            return
        self.failure = code
        self.closing = True
        self.executor.halt(self.now,code=code,revoke=True)
        self.events.append(dict(t=self.now,event='stop_requested',code=code))

    def step(self, packet):
        from bindu_contracts.contracts import Motion
        from bindu_contracts.teleoperation import ToolPoseFrame
        import numpy as np
        stamp = float(packet['stamp'])
        if packet['seq'] <= self.seq or stamp < self.now:
            raise ValueError('PHYSICS_CONTROL_UNORDERED')
        self.seq,self.now = packet['seq'],stamp
        if not packet.get('freeze_feedback') or self.feedback_stamp is None:
            self.measured,self.velocities = packet['q'],packet['qd']
            self.feedback_stamp = stamp
        if not self.initialized:
            self.initialize(packet)
        self.executor.tick(self.now)
        if self.lease is None and not self.closing:
            if self.startup_at is None:self.startup_at=self.now
            try:
                self.lease,self.epoch = self.executor.acquire(self.session.identity,self.solver.groups,2.,self.now)
            except ValueError as exc:
                if str(exc)!='FEEDBACK_NOT_STATIONARY':raise
                if self.now-self.startup_at >= 2.:self.stop('STARTUP_NOT_STATIONARY')
        metrics = None
        tool_observation = None
        if not self.closing and self.lease is not None:
            try:
                if packet.get('cancel'): self.stop('CANCELED')
                if not self.closing and self.now-self.feedback_stamp > self.cfg['feedback_max_age']:
                    self.stop('FEEDBACK_STALE')
                if not self.closing and self.executor.stop_fault:
                    self.stop(self.executor.stop_fault)
                if not self.closing and self.now >= self.next_renewal:
                    self.executor.renew(self.lease,self.epoch,self.now)
                    self.next_renewal = self.now+.3
                frame = packet.get('frame')
                if not self.closing and (frame is None or self.now-frame['stamp'] > self.cfg['input_max_age']):
                    self.stop('TOOL_INPUT_TIMEOUT')
                if not self.closing and frame is not None:
                    self.session.ingest(ToolPoseFrame(frame['source_id'],frame['seq'],frame['stamp'],
                        frame['frame_id'],tuple(frame['pose']),frame['tight']),self.now)
                if not self.closing and (self.session.source is None or self.now-self.session.stamp > self.cfg['input_max_age']):
                    self.stop('TOOL_INPUT_TIMEOUT')
                if not self.closing and self.session.grasp_generation != self.last_generation:
                    self.executor.halt(self.now)
                    self.session.reanchor();self.reference.reset()
                    self.last_generation = self.session.grasp_generation
                    self.events.append(dict(t=self.now,event='grasp_requested',mode=self.session.grasp_mode))
                if not self.closing and self.executor.stopping is None:
                    if self.session.anchor_tool is None:
                        if all(self.executor.joint_at_rest(n) for n in self.solver.names):
                            self.session.capture(self.solver.capture(self.measured,self.session.grasp_mode))
                            self.reference.capture([self.measured[n] for n in self.solver.names],self.now)
                            self.capture_mode = self.session.grasp_mode
                            self.events.append(dict(t=self.now,event='grasp_applied',mode=self.capture_mode))
                    else:
                        baseline,dt = self.reference.step(self.now)
                        if self.observer is not None:
                            nominal = np.empty((4,4))
                            q = self.solver._q(self.measured)
                            if not self.solver.lib.bq_tool_pose(self.solver.handle,self.solver._ptr(q),self.solver.nq,self.solver._ptr(nominal)):
                                raise ValueError('GRIP_LOAD_FK')
                            hand = nominal @ np.linalg.inv(self.observer.mount)
                            tool_observation = self.observer.estimate(hand,(packet.get('wrenches') or {}).get('left'),self.now)
                        positions,metrics = self.solver.solve(self.measured,self.session.target,
                            reference=baseline,dt=dt,target_stamp=self.session.stamp,tool_feedback=tool_observation)
                        self.executor.submit(Motion(str(self.seq),self.lease,self.epoch,self.profile.digest,
                            'shovel_physics',str(self.seq),'joint_target','',self.now,self.cfg['command_max_age'],
                            names=self.solver.names,positions=positions,schema_version=2,
                            resource_groups=self.solver.groups),self.now)
                        self.reference.accept(positions,self.now)
                        self.accepted += 1
            except (ValueError,RuntimeError) as exc:
                self.stop(str(exc))
        if self.closing and self.executor.stopping is None and self.hold_requested and not self.executor.stop_fault:
            # Executor has already confirmed fresh independent rest. Subsequent
            # holding drift is scored separately rather than hiding completion.
            self.stopped = True
        return dict(seq=self.seq, stamp=self.now, feedback_stamp=self.feedback_stamp,
            targets=self.targets, grasp_mode=self.capture_mode, closing=self.closing,
            stopped=self.stopped, code=self.failure, stop_fault=self.executor.stop_fault,
            accepted=self.accepted, metrics=metrics,
            tool_observation=None if tool_observation is None else tool_observation.ravel().tolist(),
            wrenches=packet.get('wrenches'),
            device_hold_requested=self.hold_requested,
            execution_feedback_stamp=self.executor.feedback.stamp,
            execution_events=[dict(stamp=e.stamp,state=e.state,code=e.code) for e in list(self.executor.events)[-4:]],
            qp_feedback_q={n:self.measured[n] for n in self.solver.model_names},
            qp_feedback_qd={n:self.velocities.get(n) for n in self.solver.model_names},
            accepted_reference=None if self.reference.positions is None else list(self.reference.positions),
            events=self.events[-3:])

    def close(self):
        self.solver.close()
        (self.output/'controller_events.json').write_text(json.dumps(self.events,indent=2)+'\n')


def worker(config, output, feedback_path):
    controller = None
    try:
        controller = PhysicsController(config,output,json.loads(Path(feedback_path).read_text()))
        print(json.dumps({'ready':True,'names':controller.solver.names,
                          'controller_sha256':hashlib.sha256(LOADED_SOURCE).hexdigest(),
                          'config_sha256':hashlib.sha256(Path(config).read_bytes()).hexdigest()}),flush=True)
        for line in sys.stdin:
            packet = json.loads(line)
            try: response = controller.step(packet)
            except Exception as exc: response = {'seq':packet.get('seq'),'fatal':type(exc).__name__+':'+str(exc)}
            response['completed_at_wall'] = time.monotonic()
            print(json.dumps(response),flush=True)
            if response.get('fatal'): break
    except Exception as exc:
        print(json.dumps({'ready':False,'fatal':type(exc).__name__+':'+str(exc)}),flush=True)
    finally:
        if controller is not None: controller.close()


if __name__ == '__main__':
    if len(sys.argv) != 5 or sys.argv[1] != '--worker':
        raise SystemExit('Private simulation worker: --worker CONFIG OUTPUT FEEDBACK_CONFIG')
    worker(sys.argv[2],sys.argv[3],sys.argv[4])
