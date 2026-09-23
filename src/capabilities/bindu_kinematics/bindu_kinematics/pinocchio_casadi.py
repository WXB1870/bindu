"""Continuous bounded IK adapted from v3.4 EEKinematicsTool.

Uses measured seed/context, joint bounds, continuity cost and optional configured
self-collision constraints. Numeric FK verifies results; failure has no command.
"""
import time
import numpy as np
from bindu_contracts.teleoperation import IKResult
from .model import ArmModel
from .collision import CollisionModel


class PinocchioCasadiIK:
    def __init__(self, config):
        import casadi as ca
        self.cfg, self.arm = config, ArmModel(config)
        self.collision = CollisionModel(self.arm, config['collision']) if 'collision' in config else None
        if self.collision and not config.get('measured_context', False):
            raise ValueError('IK_COLLISION_REQUIRES_MEASURED_CONTEXT')
        q, context, transform = self.arm.symbolic_fk(parameterize_context=True)
        self.context_fk = ca.Function('fk_context', [q, context], [transform])
        self.symbolic_fk = ca.Function('fk', [q], [self.context_fk(q, self.arm.context_default)])
        for fraction in (0., .2, .7):
            seed = (1-fraction)*self.arm.lower + fraction*self.arm.upper
            if not np.allclose(np.array(self.symbolic_fk(seed)), self.arm.fk(seed), atol=1e-7):
                raise ValueError('MODEL_FK_MISMATCH')
        self.opti = opti = ca.Opti()
        self.q = opti.variable(len(self.arm.names))
        self.seed = opti.parameter(len(self.arm.names))
        self.context = opti.parameter(len(self.arm.context_names))
        self.target = opti.parameter(4, 4)
        pose = self.context_fk(self.q, self.context)
        loss = (500*ca.sumsqr(pose[:3, 3]-self.target[:3, 3]) +
                25*ca.sumsqr(pose[:3, :3]-self.target[:3, :3]) +
                config['smooth_weight']*ca.sumsqr(self.q-self.seed))
        opti.minimize(loss)
        opti.subject_to(opti.bounded(self.arm.lower, self.q, self.arm.upper))
        if self.collision:
            # Small numerical buffer; numeric FK still enforces the full margin.
            opti.subject_to(self.collision.symbolic(self.q, self.context) >= 1e-6)
        opti.solver('ipopt', {'print_time': False},
                    {'print_level': 0, 'sb': 'yes', 'max_iter': 60, 'tol': 1e-5,
                     'max_cpu_time': config['solve_timeout']})
        # Warm the solver before the runtime exposes readiness.
        seed = np.clip(np.zeros(len(self.arm.names)), self.arm.lower, self.arm.upper)
        self._solve(seed, self.arm.fk(seed))

    def _solve(self, seed, target, context=None):
        self.opti.set_initial(self.q, seed)
        self.opti.set_value(self.seed, seed)
        self.opti.set_value(self.context, self.arm.context_default if context is None else context)
        self.opti.set_value(self.target, target)
        return np.asarray(self.opti.solve().value(self.q)).reshape(-1)

    def solve(self, request):
        start = time.monotonic()
        try:
            seed = np.asarray(request.seed, dtype=float)
            if (request.names != self.arm.names or seed.shape != (len(self.arm.names),) or
                    not np.isfinite(seed).all() or (seed < self.arm.lower).any() or (seed > self.arm.upper).any()):
                raise ValueError('IK_INVALID_SEED')
            context = np.asarray(request.context if request.context else self.arm.context_default, dtype=float)
            if ((self.cfg.get('measured_context', False) and not request.context) or
                    context.shape != (len(self.arm.context_names),) or not np.isfinite(context).all() or
                    (context < self.arm.full.lowerPositionLimit[self.arm.context_indices]).any() or
                    (context > self.arm.full.upperPositionLimit[self.arm.context_indices]).any()):
                raise ValueError('IK_INVALID_CONTEXT')
            target = np.asarray(request.target).reshape(4, 4) if request.target else self.arm.fk(seed, context)
            if (not np.isfinite(target).all() or not np.allclose(target[3], [0, 0, 0, 1]) or
                    not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-3) or
                    not np.isclose(np.linalg.det(target[:3, :3]), 1., atol=1e-3)):
                raise ValueError('IK_INVALID_TARGET')
            if self.collision and not self.collision.safe(seed, context):
                raise ValueError('IK_COLLISION_SEED')
            q = self._solve(seed, target, context) if request.target else seed
            actual = self.arm.fk(q, context)
            pe = float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))
            cosine = np.clip((np.trace(target[:3, :3].T @ actual[:3, :3])-1)/2, -1., 1.)
            re = float(np.arccos(cosine))
            if not np.isfinite(q).all() or (q < self.arm.lower-1e-8).any() or (q > self.arm.upper+1e-8).any():
                raise ValueError('IK_JOINT_LIMIT')
            if pe > self.cfg['position_tolerance'] or re > self.cfg['rotation_tolerance']:
                raise ValueError('IK_RESIDUAL')
            if np.max(np.abs(q-seed)) > self.cfg['max_joint_step']:
                raise ValueError('IK_DISCONTINUITY')
            if self.collision and not self.collision.segment_safe(seed, q, context):
                raise ValueError('IK_COLLISION_PATH')
            elapsed = time.monotonic()-start
            if elapsed > self.cfg['solve_timeout']:
                raise ValueError('IK_TIMEOUT')
            return IKResult(request, True, 'OK', tuple(q), tuple(actual.ravel()), pe, re, elapsed)
        except (ValueError, RuntimeError) as exc:
            code = str(exc) if isinstance(exc, ValueError) else 'IK_SOLVER_FAILED'
            return IKResult(request, False, code, elapsed=time.monotonic()-start)
