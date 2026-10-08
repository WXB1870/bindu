"""Optional native rigid-tool QP with named, model-derived joint layout."""
import ctypes as c
import math
import time
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_prefix


ABI_VERSION = 1


class NativeConfig(c.Structure):
    """Layout from bindu_bimanual_qp/c_api.h; defaults belong to the native core."""
    _fields_ = [('abi_version', c.c_uint32), ('struct_size', c.c_uint32)] + [
        (name, c.c_double) for name in (
            'tool_gain relative_gain posture_gain posture_weight shared_target_weight '
            'shared_target_gain regularization max_linear_speed max_angular_speed '
            'max_relative_error limit_margin damper_gain equality_tolerance').split()]


def check_abi(lib):
    try:
        version = lib.bq_abi_version
    except AttributeError as exc:
        raise ValueError('BIMANUAL_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
    version.argtypes, version.restype = [], c.c_uint32
    actual = version()
    if actual != ABI_VERSION:
        raise ValueError(f'BIMANUAL_ABI_MISMATCH: expected {ABI_VERSION}, got {actual}')


class BimanualQP:
    def __init__(self, config, profile):
        self.cfg = config
        self.captured = False
        self.handle = None
        self.groups = tuple(config['resource_groups'])
        if (len(self.groups) not in (2,3) or len(set(self.groups)) != len(self.groups) or
                any(group not in profile.groups for group in self.groups)):
            raise ValueError('BIMANUAL_RESOURCE_LAYOUT')
        self.names = tuple(name for group in self.groups for name in profile.groups[group])
        self.max_interval = float(config.get('max_integration_interval', .1))
        self.max_reference_error = float(config.get('max_reference_error', .15))
        if not all(math.isfinite(float(v)) and float(v) > 0 for v in
                   (config['sample_period'], config['max_relative_error'],
                    config['max_linear_speed'], config['max_angular_speed'],
                    config['solve_timeout_us'], self.max_interval, self.max_reference_error)) or (
                        config['sample_period'] > self.max_interval or self.max_interval > .1):
            raise ValueError('BIMANUAL_INVALID_LIMIT')
        path = Path(config.get('library') or Path(get_package_prefix('bindu_bimanual_qp')) /
                    'lib/libbindu_bimanual_qp.so')
        self.lib = lib = c.CDLL(str(path))
        # Check before resolving or calling any ABI-dependent function.
        check_abi(lib)
        p, i, d = c.c_void_p, c.c_int, c.c_double
        string = c.c_char_p
        array = c.POINTER(d)
        for name, args, result in (
            ('bq_config_defaults', [c.POINTER(NativeConfig), c.c_uint32], i),
            ('bq_create', [string]*6, p), ('bq_destroy', [p], None),
            ('bq_nq', [p], i), ('bq_nactive', [p], i), ('bq_nmodel', [p], i),
            ('bq_model_name', [p,i], string), ('bq_active_name', [p,i], string),
            ('bq_error', [p], string), ('bq_limits', [p,array,array,array,i], i),
            ('bq_set_limits', [p,array,array,array,i], i),
            ('bq_capture', [p,array,i,array,c.POINTER(NativeConfig)], i),
            ('bq_shared_targets', [p,array,i], i),
            ('bq_solve', [p,array,i,array,i,array,d,array,array], i),
            ('bq_tool_pose', [p,array,i,array], i),
            ('bq_relative_pose', [p,array,i,array], i)):
            fn = getattr(lib, name)
            fn.argtypes, fn.restype = args, result
        self.native_config = NativeConfig()
        if not lib.bq_config_defaults(c.byref(self.native_config), c.sizeof(NativeConfig)):
            raise ValueError('BIMANUAL_CONFIG_ABI_MISMATCH')
        for name, _ in NativeConfig._fields_[2:]:
            if name in config:
                setattr(self.native_config, name, float(config[name]))
        def encoded(value): return str(value).encode()
        shared = profile.groups[self.groups[0]] if len(self.groups) == 3 else ()
        self.shared_names = tuple(shared)
        left = profile.groups[self.groups[-2]]
        right = profile.groups[self.groups[-1]]
        self.handle = lib.bq_create(encoded(config['urdf']), encoded(config['left_frame']),
            encoded(config['right_frame']), encoded(','.join(shared)),
            encoded(','.join(left)), encoded(','.join(right)))
        if not self.handle:
            raise ValueError('BIMANUAL_MODEL_INVALID')
        self.nq = lib.bq_nq(self.handle)
        self.model_names = tuple(lib.bq_model_name(self.handle,k).decode()
                                 for k in range(lib.bq_nmodel(self.handle)))
        active = tuple(lib.bq_active_name(self.handle,k).decode()
                       for k in range(lib.bq_nactive(self.handle)))
        if active != self.names or self.nq != len(self.model_names):
            self.close()
            raise ValueError('BIMANUAL_MODEL_LAYOUT')
        lower = np.empty(len(active)); upper = np.empty(len(active)); speed = np.empty(len(active))
        if not lib.bq_limits(self.handle, self._ptr(lower), self._ptr(upper),
                             self._ptr(speed), len(active)):
            self.close()
            raise ValueError('BIMANUAL_MODEL_LIMITS')
        for n, lo, hi, v in zip(active, lower, upper, speed):
            plo, phi = profile.limits[n]
            if (plo < lo-1e-6 or phi > hi+1e-6 or
                    profile.speed(n) > v+1e-6):
                self.close()
                raise ValueError('BIMANUAL_PROFILE_EXCEEDS_MODEL:'+n)
        # Native bounds use the intersection of profile and URDF. The Python
        # postcheck remains an independent check against the executor's profile.
        configured_speed = np.minimum(speed, [profile.speed(n) for n in active])
        lower = np.maximum(lower, [profile.limits[n][0] for n in active])
        upper = np.minimum(upper, [profile.limits[n][1] for n in active])
        if not lib.bq_set_limits(self.handle, self._ptr(lower), self._ptr(upper),
                                self._ptr(configured_speed), len(active)):
            error=self._error()
            self.close()
            raise ValueError('BIMANUAL_LIMIT_CONFIG:'+error)
        try:
            self.tool = self._transform(config['left_to_tool'])
            self.expected_relative = None
            if 'right_to_tool' in config:
                right_tool = self._transform(config['right_to_tool'])
                self.expected_relative = self.tool @ np.linalg.inv(right_tool)
                self.grasp_position_tolerance = float(config['grasp_position_tolerance_m'])
                self.grasp_rotation_tolerance = float(config['grasp_rotation_tolerance_rad'])
                if not all(math.isfinite(v) and v > 0 for v in
                           (self.grasp_position_tolerance, self.grasp_rotation_tolerance)):
                    raise ValueError('BIMANUAL_GRASP_TOLERANCE')
        except (KeyError, TypeError, ValueError):
            self.close()
            raise
        self.limits = profile.limits
        self.speed = {n: min(profile.speed(n), float(v)) for n,v in zip(active,speed)}

    def close(self):
        self.captured = False
        if getattr(self, 'handle', None):
            self.lib.bq_destroy(self.handle)
            self.handle = None

    def __del__(self):
        self.close()

    def _q(self, feedback):
        try:
            q = np.asarray([feedback[n] for n in self.model_names], dtype=np.float64)
        except KeyError as exc:
            raise ValueError('BIMANUAL_FEEDBACK_LAYOUT') from exc
        if not np.isfinite(q).all():
            raise ValueError('BIMANUAL_FEEDBACK_NONFINITE')
        return q

    @staticmethod
    def _ptr(values):
        return values.ctypes.data_as(c.POINTER(c.c_double))

    def _error(self):
        return self.lib.bq_error(self.handle).decode()

    @staticmethod
    def _transform(values):
        transform = np.asarray(values, dtype=np.float64).reshape(4,4)
        rotation = transform[:3,:3]
        if (not np.isfinite(transform).all() or not np.allclose(transform[3], [0,0,0,1]) or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or
                np.linalg.det(rotation) < .999):
            raise ValueError('BIMANUAL_TOOL_TRANSFORM')
        return transform

    def capture(self, feedback):
        self.captured = False
        if not self.handle:
            raise ValueError('BIMANUAL_CLOSED')
        q = self._q(feedback)
        measured = np.empty((4,4), dtype=np.float64)
        if not self.lib.bq_relative_pose(self.handle, self._ptr(q), self.nq, self._ptr(measured)):
            raise ValueError('BIMANUAL_GRASP_POSE:'+self._error())
        distance = float(np.linalg.norm(measured[:3,3]))
        low, high = self.cfg['flange_distance_range']
        if not low <= distance <= high:
            raise ValueError('BIMANUAL_GRASP_GEOMETRY')
        if self.expected_relative is not None:
            error = np.linalg.inv(self.expected_relative) @ measured
            angle = math.acos(float(np.clip((np.trace(error[:3,:3])-1.)/2., -1., 1.)))
            if (np.linalg.norm(error[:3,3]) > self.grasp_position_tolerance or
                    angle > self.grasp_rotation_tolerance):
                raise ValueError('BIMANUAL_GRASP_POSE')
        if not self.lib.bq_capture(self.handle, self._ptr(q), self.nq, self._ptr(self.tool),
                                  c.byref(self.native_config)):
            raise ValueError('BIMANUAL_CAPTURE:'+self._error())
        out = np.empty((4,4), dtype=np.float64)
        if not self.lib.bq_tool_pose(self.handle, self._ptr(q), self.nq, self._ptr(out)):
            raise ValueError('BIMANUAL_TOOL_POSE:'+self._error())
        self.captured = True
        try:
            self.set_shared_targets(self.cfg.get('shared_targets'))
        except (ValueError, TypeError):
            self.captured = False
            raise
        return out

    def set_shared_targets(self, targets=None):
        """Optional named shared-joint posture objective; None clears it."""
        if not self.captured:
            raise ValueError('BIMANUAL_NOT_CAPTURED')
        if targets is None:
            ok = self.lib.bq_shared_targets(self.handle, None, 0)
        else:
            if not isinstance(targets, dict) or set(targets) != set(self.shared_names):
                raise ValueError('BIMANUAL_SHARED_TARGET_LAYOUT')
            values = np.array([targets[n] for n in self.shared_names], dtype=np.float64)
            ok = self.lib.bq_shared_targets(self.handle, self._ptr(values), len(values))
        if not ok:
            raise ValueError('BIMANUAL_SHARED_TARGET:'+self._error())

    def solve(self, feedback, target, *, reference, dt):
        """Propose a step; caller commits reference/time only after acceptance.

        Feedback/Jacobians remain measured. dt is explicit, not inferred from VR
        frequency. A large tracking error requires hold and a fresh capture;
        silently realigning a moving reference would introduce a position jump.
        """
        if not self.captured:
            raise ValueError('BIMANUAL_NOT_CAPTURED')
        try:
            return self._solve(feedback, target, reference, float(dt))
        except (ValueError, TypeError):
            self.captured = False
            raise

    def _solve(self, feedback, target, reference, dt):
        if not math.isfinite(dt) or not 0 < dt <= self.max_interval:
            raise ValueError('BIMANUAL_INTEGRATION_INTERVAL')
        q = self._q(feedback)
        reference = np.ascontiguousarray(reference, dtype=np.float64)
        if reference.shape != (len(self.names),) or not np.isfinite(reference).all():
            raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
        measured = np.array([feedback[n] for n in self.names])
        if np.max(np.abs(reference-measured)) > self.max_reference_error:
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
        target = np.ascontiguousarray(target, dtype=np.float64).reshape(4,4)
        out = np.empty(len(self.names), dtype=np.float64)
        metrics = np.empty(3, dtype=np.float64)
        started = time.perf_counter()
        if not self.lib.bq_solve(self.handle, self._ptr(q), self.nq, self._ptr(reference),
                len(self.names), self._ptr(target), dt, self._ptr(out), self._ptr(metrics)):
            raise ValueError('BIMANUAL_QP:'+self._error())
        if (time.perf_counter()-started)*1e6 > float(self.cfg['solve_timeout_us']):
            raise ValueError('BIMANUAL_QP_TIMEOUT')
        if not np.isfinite(out).all() or not np.isfinite(metrics).all():
            raise ValueError('BIMANUAL_NONFINITE_RESULT')
        if np.max(np.abs(out-measured)) > self.max_reference_error:
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
        for name, old, new in zip(self.names, reference, out):
            lo, hi = self.limits[name]
            if not lo <= new <= hi or abs(new-old) > self.speed[name]*dt+1e-6:
                raise ValueError('BIMANUAL_POSITION_LIMIT:'+name)
        return tuple(float(v) for v in out), tuple(float(v) for v in metrics)
