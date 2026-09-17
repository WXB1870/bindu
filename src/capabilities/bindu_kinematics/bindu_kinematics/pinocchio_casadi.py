"""Continuous bounded IK adapted from v3.4 EEKinematicsTool.

Uses last measured/accepted pose as seed, joint bounds and continuity cost.
Residuals are verified with independent numeric FK; failure has no command.
"""
import time
import numpy as np
from bindu_contracts.teleoperation import IKResult
from .model import ArmModel


class PinocchioCasadiIK:
    def __init__(self, config):
        import casadi as ca
        self.cfg, self.arm = config, ArmModel(config)
        q, transform = self.arm.symbolic_fk()
        self.symbolic_fk = ca.Function('fk', [q], [transform])
        for fraction in (0., .2, .7):
            seed = (1-fraction)*self.arm.lower + fraction*self.arm.upper
            if not np.allclose(np.array(self.symbolic_fk(seed)), self.arm.fk(seed), atol=1e-7):
                raise ValueError('MODEL_FK_MISMATCH')
        self.opti = opti = ca.Opti()
        self.q = opti.variable(len(self.arm.names))
        self.seed = opti.parameter(len(self.arm.names))
        self.target = opti.parameter(4, 4)
        pose = self.symbolic_fk(self.q)
        loss = (500*ca.sumsqr(pose[:3, 3]-self.target[:3, 3]) +
                25*ca.sumsqr(pose[:3, :3]-self.target[:3, :3]) +
                config['smooth_weight']*ca.sumsqr(self.q-self.seed))
        opti.minimize(loss)
        opti.subject_to(opti.bounded(self.arm.lower, self.q, self.arm.upper))
        opti.solver('ipopt', {'print_time': False},
                    {'print_level': 0, 'sb': 'yes', 'max_iter': 60, 'tol': 1e-5,
                     'max_cpu_time': config['solve_timeout']})
        # Warm the solver before the runtime exposes readiness.
        seed = np.clip(np.zeros(len(self.arm.names)), self.arm.lower, self.arm.upper)
        self._solve(seed, self.arm.fk(seed))

    def _solve(self, seed, target):
        self.opti.set_initial(self.q, seed)
        self.opti.set_value(self.seed, seed)
        self.opti.set_value(self.target, target)
        return np.asarray(self.opti.solve().value(self.q)).reshape(-1)

    def solve(self, request):
        start = time.monotonic()
        try:
            seed = np.asarray(request.seed, dtype=float)
            if (request.names != self.arm.names or seed.shape != (len(self.arm.names),) or
                    not np.isfinite(seed).all() or (seed < self.arm.lower).any() or (seed > self.arm.upper).any()):
                raise ValueError('IK_INVALID_SEED')
            target = np.asarray(request.target).reshape(4, 4) if request.target else self.arm.fk(seed)
            if (not np.isfinite(target).all() or not np.allclose(target[3], [0, 0, 0, 1]) or
                    not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-3) or
                    not np.isclose(np.linalg.det(target[:3, :3]), 1., atol=1e-3)):
                raise ValueError('IK_INVALID_TARGET')
            q = self._solve(seed, target) if request.target else seed
            actual = self.arm.fk(q)
            pe = float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))
            cosine = np.clip((np.trace(target[:3, :3].T @ actual[:3, :3])-1)/2, -1., 1.)
            re = float(np.arccos(cosine))
            if not np.isfinite(q).all() or (q < self.arm.lower-1e-8).any() or (q > self.arm.upper+1e-8).any():
                raise ValueError('IK_JOINT_LIMIT')
            if pe > self.cfg['position_tolerance'] or re > self.cfg['rotation_tolerance']:
                raise ValueError('IK_RESIDUAL')
            if np.max(np.abs(q-seed)) > self.cfg['max_joint_step']:
                raise ValueError('IK_DISCONTINUITY')
            elapsed = time.monotonic()-start
            if elapsed > self.cfg['solve_timeout']:
                raise ValueError('IK_TIMEOUT')
            return IKResult(request, True, 'OK', tuple(q), tuple(actual.ravel()), pe, re, elapsed)
        except (ValueError, RuntimeError) as exc:
            code = str(exc) if isinstance(exc, ValueError) else 'IK_SOLVER_FAILED'
            return IKResult(request, False, code, elapsed=time.monotonic()-start)
