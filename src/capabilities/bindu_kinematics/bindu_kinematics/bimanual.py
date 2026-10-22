"""Optional native rigid/axial tool QP with named, model-derived joint layout."""
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
        self.reference_kinematics = config.get('reference_kinematics', False)
        if type(self.reference_kinematics) is not bool:
            raise ValueError('BIMANUAL_REFERENCE_KINEMATICS_CONFIG')
        self.tracking_feedback = config.get('tracking_feedback', 'measured')
        if not isinstance(self.tracking_feedback, str) or self.tracking_feedback not in ('measured', 'reference'):
            raise ValueError('BIMANUAL_TRACKING_FEEDBACK_CONFIG')
        if self.tracking_feedback == 'reference' and not self.reference_kinematics:
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_REQUIRES_REFERENCE_KINEMATICS')
        self.feedforward = config.get('tool_velocity_feedforward', False)
        if type(self.feedforward) is not bool:
            raise ValueError('BIMANUAL_FEEDFORWARD_CONFIG')
        self.tracking = self.feedforward or self.reference_kinematics
        self.target_sample = None
        self.last_timing = None
        self.last_geometry_step = None
        self.geometry_step_backtracking=config.get('geometry_step_backtracking',False)
        if type(self.geometry_step_backtracking) is not bool:
            raise ValueError('BIMANUAL_GEOMETRY_BACKTRACKING_CONFIG')
        self.grasp_reference = None
        self.handle = None
        motion_weight = config.get('shared_motion_weight', 0.)
        if (isinstance(motion_weight, bool) or not isinstance(motion_weight, (int, float)) or
                not math.isfinite(motion_weight) or motion_weight < 0):
            raise ValueError('BIMANUAL_SHARED_MOTION_WEIGHT')
        self.shared_motion_weight = float(motion_weight)
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
        if self.shared_motion_weight > 0:
            try:fn=lib.bq_shared_motion_weight
            except AttributeError as exc:
                raise ValueError('BIMANUAL_SHARED_MOTION_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
            fn.argtypes,fn.restype=[c.c_void_p,c.c_double],c.c_int
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
        if self.tracking:
            try:
                fn = lib.bq_solve_tracking
            except AttributeError as exc:
                raise ValueError('BIMANUAL_TRACKING_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
            fn.argtypes, fn.restype = [p,array,i,array,i,array,array,array,c.c_double,array,array], i
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
            self.posture_policy = config.get('posture_policy')
            self.posture_reference = None
            self.posture_reference_margin = float(config.get('posture_reference_margin_rad',0.))
            if not math.isfinite(self.posture_reference_margin) or self.posture_reference_margin < 0:
                raise ValueError('BIMANUAL_POSTURE_REFERENCE_MARGIN')
            self.posture_lower, self.posture_upper = lower.copy(), upper.copy()
            if self.posture_policy is not None:
                policy = self.posture_policy
                if (not isinstance(policy,dict) or set(policy) != {'reference','position_bounds'} or
                        policy['reference'] != 'initial_capture' or not isinstance(policy['position_bounds'],dict) or
                        not set(policy['position_bounds']).issubset(active)):
                    raise ValueError('BIMANUAL_POSTURE_POLICY')
                for name, bounds in policy['position_bounds'].items():
                    j=active.index(name)
                    values=np.asarray(bounds,dtype=float)
                    if (values.shape!=(2,) or not np.isfinite(values).all() or
                            not lower[j] <= values[0] < values[1] <= upper[j]):
                        raise ValueError('BIMANUAL_POSTURE_BOUNDS:'+name)
                    self.posture_lower[j],self.posture_upper[j]=values
                try:
                    fn=lib.bq_posture_policy
                except AttributeError as exc:
                    raise ValueError('BIMANUAL_POSTURE_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
                fn.argtypes,fn.restype=[p,array,array,array,i],i
            if self.posture_reference_margin > 0:
                if self.posture_policy is None:raise ValueError('BIMANUAL_POSTURE_REFERENCE_MARGIN')
                try:fn=lib.bq_posture_reference_margin
                except AttributeError as exc:
                    raise ValueError('BIMANUAL_POSTURE_MARGIN_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
                fn.argtypes,fn.restype=[p,d],i
            self.tool = self._transform(config['left_to_tool'])
            self.axial = config.get('right_axial_grasp')
            self.grasp_mode = 'rigid'
            if self.axial is not None:
                try:
                    fn = lib.bq_capture_axial
                except AttributeError as exc:
                    raise ValueError('BIMANUAL_AXIAL_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
                fn.argtypes, fn.restype = [p,array,i,array,array,array,c.POINTER(NativeConfig)], i
                right_tool = self._transform(config['right_to_tool'])
                axis = np.asarray(self.axial['axis_in_tool'], dtype=float)
                point = np.asarray(self.axial['grip_in_right_flange_m'], dtype=float)
                if (axis.shape != (3,) or point.shape != (3,) or
                        not np.isfinite(np.r_[axis,point]).all() or
                        not np.isclose(np.linalg.norm(axis), 1., atol=1e-8)):
                    raise ValueError('BIMANUAL_AXIAL_GEOMETRY')
                helper = np.eye(3)[np.argmin(abs(axis))]
                transverse = np.cross(helper,axis); transverse /= np.linalg.norm(transverse)
                self.tool_grip = np.eye(4)
                self.tool_grip[:3,:3] = np.column_stack((transverse,np.cross(axis,transverse),axis))
                self.tool_grip[:3,3] = (np.linalg.inv(right_tool) @ np.r_[point,1.])[:3]
                self.right_grip = right_tool @ self.tool_grip
            self.expected_relative = None
            if 'right_to_tool' in config:
                right_tool = self._transform(config['right_to_tool'])
                self.expected_relative = self.tool @ np.linalg.inv(right_tool)
                self.grasp_position_tolerance = float(config['grasp_position_tolerance_m'])
                self.grasp_rotation_tolerance = float(config['grasp_rotation_tolerance_rad'])
                if not all(math.isfinite(v) and v > 0 for v in
                           (self.grasp_position_tolerance, self.grasp_rotation_tolerance)):
                    raise ValueError('BIMANUAL_GRASP_TOLERANCE')
            point=config.get('tool_tracking_point_in_tool_m')
            if point is None and config.get('vr_tool_mapping')=='hand_line_midpoint':
                point=np.mean([(np.linalg.inv(self._transform(config[side+'_to_tool'])) @
                    np.r_[config['demo'][key],1.])[:3] for side,key in
                    (('left','grip_offset_m'),('right','right_grip_offset_m'))],axis=0)
            self.tracking_point=np.ascontiguousarray(np.zeros(3) if point is None else point,dtype=np.float64)
            if self.tracking_point.shape!=(3,) or not np.isfinite(self.tracking_point).all() or np.linalg.norm(self.tracking_point)>3.:
                raise ValueError('BIMANUAL_TRACKING_POINT')
        except (KeyError, TypeError, ValueError):
            self.close()
            raise
        self.collision_check = None
        if config.get('collision_scope') == 'configured_tool_geometry':
            try:
                from .tool_geometry import make_collision_checker
                recipe = dict(config['demo'])
                recipe['left_grip_in_tool_m'] = (np.linalg.inv(self.tool) @
                    np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
                self.collision_check = make_collision_checker(
                    Path(config['urdf']).parent/recipe['display_urdf'],recipe,
                    conservative_convex=True,validation_only=True)
                self.collision_step = float(config.get('collision_joint_step_rad',.01))
                if not math.isfinite(self.collision_step) or not 0 < self.collision_step <= .01:
                    raise ValueError('BIMANUAL_COLLISION_STEP')
            except Exception:
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
        # data_as casts a c_void_p whose _objects dictionary refers back to
        # itself. Every call leaves cyclic garbage and can trigger a full GC
        # inside a timed solve. Cast the address and retain the array explicitly.
        pointer = c.cast(values.ctypes.data, c.POINTER(c.c_double))
        pointer._arr = values
        return pointer

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

    def capture(self, feedback, grasp_mode='rigid'):
        self.captured = False
        self.target_sample = None
        if not self.handle:
            raise ValueError('BIMANUAL_CLOSED')
        if grasp_mode not in ('rigid', 'right_axial') or grasp_mode == 'right_axial' and self.axial is None:
            raise ValueError('BIMANUAL_GRASP_MODE')
        q = self._q(feedback)
        measured = np.empty((4,4), dtype=np.float64)
        if not self.lib.bq_relative_pose(self.handle, self._ptr(q), self.nq, self._ptr(measured)):
            raise ValueError('BIMANUAL_GRASP_POSE:'+self._error())
        distance = float(np.linalg.norm(measured[:3,3]))
        low, high = self.cfg['flange_distance_range']
        if self.axial is None and not low <= distance <= high:
            raise ValueError('BIMANUAL_GRASP_GEOMETRY')
        if self.expected_relative is not None:
            error = np.linalg.inv(self.expected_relative) @ measured
            angle = math.acos(float(np.clip((np.trace(error[:3,:3])-1.)/2., -1., 1.)))
            distance_error = np.linalg.norm(error[:3,3])
            if self.axial is not None:
                # Re-lock at the measured roll angle after slip; never restore
                # the old 6D reference. Grasp point and shaft direction remain
                # tied to calibrated geometry even across sessions.
                grip = np.linalg.inv(self.tool) @ measured @ self.right_grip
                distance_error = np.linalg.norm(grip[:3,3]-self.tool_grip[:3,3])
                angle = math.acos(float(np.clip(grip[:3,2] @ self.tool_grip[:3,2], -1., 1.)))
            if (distance_error > self.grasp_position_tolerance or
                    angle > self.grasp_rotation_tolerance):
                raise ValueError('BIMANUAL_GRASP_POSE')
        if grasp_mode == 'right_axial':
            ok = self.lib.bq_capture_axial(self.handle, self._ptr(q), self.nq, self._ptr(self.tool),
                self._ptr(self.right_grip), self._ptr(self.tool_grip), c.byref(self.native_config))
        else:
            ok = self.lib.bq_capture(self.handle, self._ptr(q), self.nq, self._ptr(self.tool),
                                     c.byref(self.native_config))
        if not ok:
            raise ValueError('BIMANUAL_CAPTURE:'+self._error())
        self._apply_shared_motion_weight()
        if np.any(self.tracking_point):
            try:fn=self.lib.bq_tracking_point
            except AttributeError as exc:
                raise ValueError('BIMANUAL_TRACKING_POINT_ABI_MISSING: rebuild bindu_bimanual_qp') from exc
            fn.argtypes,fn.restype=[c.c_void_p,c.POINTER(c.c_double)],c.c_int
            if not fn(self.handle,self._ptr(self.tracking_point)):
                raise ValueError('BIMANUAL_TRACKING_POINT:'+self._error())
        posture_reference = self.posture_reference
        if self.posture_policy is not None:
            if posture_reference is None:
                posture_reference = np.array([feedback[n] for n in self.names],dtype=np.float64)
            if not self.lib.bq_posture_policy(self.handle,self._ptr(posture_reference),
                    self._ptr(self.posture_lower),self._ptr(self.posture_upper),len(self.names)):
                raise ValueError('BIMANUAL_POSTURE_POLICY:'+self._error())
            if self.posture_reference_margin > 0 and not self.lib.bq_posture_reference_margin(self.handle,self.posture_reference_margin):
                raise ValueError('BIMANUAL_POSTURE_REFERENCE_MARGIN:'+self._error())
        self.grasp_mode = grasp_mode
        self.grasp_reference = measured.copy()
        out = np.empty((4,4), dtype=np.float64)
        if not self.lib.bq_tool_pose(self.handle, self._ptr(q), self.nq, self._ptr(out)):
            raise ValueError('BIMANUAL_TOOL_POSE:'+self._error())
        self.captured = True
        try:
            self.check_path(feedback, [feedback[n] for n in self.names])
            self.set_shared_targets(self.cfg.get('shared_targets'))
        except (ValueError, TypeError):
            self.captured = False
            raise
        # Retain only a successfully captured preference across grasp changes;
        # the measured tool/grasp reference is still rebuilt on every capture.
        if self.posture_policy is not None:
            self.posture_reference = posture_reference.copy()
        return out

    def check_path(self, feedback, *waypoints):
        """Fail closed on sampled feedback/reference/candidate geometry.

        Joint-space segments are sampled at <= configured step; this is a
        rejection guard, not trajectory avoidance or a swept-volume guarantee.
        """
        if self.collision_check is None:
            return
        base = self._q(feedback)
        before = np.array([feedback[n] for n in self.names])
        samples=[]
        indices=[self.model_names.index(n) for n in self.names]
        for end in waypoints:
            end=np.asarray(end,dtype=float)
            count=max(1,int(np.ceil(np.max(np.abs(end-before))/self.collision_step)))
            for fraction in np.linspace(0.,1.,count+1):
                q=base.copy(); q[indices]=before+fraction*(end-before)
                if samples and np.max(np.abs(q-last_q)) < 1e-12:
                    continue
                last_q=q
                tool=np.empty((4,4),dtype=np.float64)
                if not self.lib.bq_tool_pose(self.handle,self._ptr(q),self.nq,self._ptr(tool)):
                    raise ValueError('BIMANUAL_COLLISION_FK:'+self._error())
                if self.expected_relative is not None and self.captured:
                    relative=np.empty((4,4),dtype=np.float64)
                    if not self.lib.bq_relative_pose(self.handle,self._ptr(q),self.nq,self._ptr(relative)):
                        raise ValueError('BIMANUAL_COLLISION_FK:'+self._error())
                    distance,angle=self._grip_error(relative)
                    if distance>self.grasp_position_tolerance or angle>self.grasp_rotation_tolerance:
                        raise ValueError(f'GRASP_CLOSURE: position_m={distance:.6g} rotation_rad={angle:.6g}')
                samples.append({'t':float(len(samples)),'q':dict(zip(self.model_names,q)),
                                'tool':tool})
            before=end
        self._validate_geometry(self.collision_check,samples)

    def _validate_geometry(self, checker, samples):
        if getattr(checker,'validation_only',False):
            checker(samples,deadline=getattr(self,'_geometry_deadline',None))
        else:
            checker(samples)

    def _grip_error(self, relative):
        error=np.linalg.inv(self.grasp_reference) @ relative
        angle=math.acos(float(np.clip((np.trace(error[:3,:3])-1.)/2.,-1.,1.)))
        distance=np.linalg.norm(relative[:3,3]-self.expected_relative[:3,3])
        if self.axial is not None:
            actual=np.linalg.inv(self.tool) @ relative @ self.right_grip
            distance=np.linalg.norm(actual[:3,3]-self.tool_grip[:3,3])
            if self.grasp_mode=='right_axial':
                angle=math.acos(float(np.clip(actual[:3,2] @ self.tool_grip[:3,2],-1.,1.)))
        return distance,angle

    def checked_geometry_step(self, feedback, reference, candidate, *, deadline=None, independent=False):
        """Bound only a rejected candidate; never skip measured/path validation."""
        from .tool_geometry import GeometryBudgetExceeded
        check=self.check_independent_path if independent else self.check_path
        failures=('GRASP_PENETRATION:','GRASP_CLOSURE:','SELF_COLLISION:','PAN_CLEARANCE:','SHAFT_CLEARANCE:','BIN_CLEARANCE:')
        if not self.geometry_step_backtracking:
            check(feedback,candidate,reference)
            return candidate,1.
        # Validate a safe fallback BEFORE spending time on candidate search.
        # Actual collisions and an unsafe reference still fail this operation.
        check(feedback,reference)
        delta=candidate-reference
        limited=reference.copy();accepted_scale=0.;budget_limited=False;reason=None
        self._geometry_deadline=None if deadline is None else deadline-.003
        try:
            for scale in (1.,.5,.25,.125,.0625,.03125):
                if self._geometry_deadline is not None and time.perf_counter()>=self._geometry_deadline:
                    budget_limited=True;break
                limited=reference+scale*delta
                try:check(feedback,limited,reference)
                except GeometryBudgetExceeded:
                    budget_limited=True;break
                except ValueError as exc:
                    if not str(exc).startswith(failures):raise
                    if reason is None:reason=str(exc)
                    continue
                accepted_scale=scale;break
        finally:
            self._geometry_deadline=None
        if accepted_scale==1.:return candidate,1.
        if not accepted_scale:limited=reference.copy()
        self.last_geometry_step=dict(scale=accepted_scale,first_rejection=reason or 'GEOMETRY_VALIDATION_BUDGET',
            attempted_joint_step_rad=float(np.max(np.abs(delta))),
            proposed_joint_step_rad=float(np.max(np.abs(limited-reference))),
            search_limited_by_budget=budget_limited)
        return limited,accepted_scale

    def check_loaded_prepare(self, feedback, targets):
        """Validate a proposed joint path with the tool and captured rigid grasp.

        This is a discrete rejection guard, not a planner. A joint-space path
        that breaks the measured two-hand geometry is rejected before motion.
        """
        self.capture(feedback,'rigid')
        reference=np.linalg.inv(self.grasp_reference)
        before=self._q(feedback)
        after=self._q({n:targets.get(n,feedback[n]) for n in self.model_names})
        if any(not self.limits[n][0]<=value<=self.limits[n][1] for n,value in zip(self.model_names,after)):
            raise ValueError('PREPARE_JOINT_LIMIT')
        count=max(1,int(np.ceil(np.max(np.abs(after-before))/min(self.collision_step,.005))))
        samples=[]
        for fraction in np.linspace(0.,1.,count+1):
            q=before+fraction*(after-before)
            relative=np.empty((4,4),dtype=np.float64)
            tool=np.empty((4,4),dtype=np.float64)
            if (not self.lib.bq_relative_pose(self.handle,self._ptr(q),self.nq,self._ptr(relative)) or
                    not self.lib.bq_tool_pose(self.handle,self._ptr(q),self.nq,self._ptr(tool))):
                raise ValueError('PREPARE_TOOL_FK')
            error=reference @ relative
            angle=math.acos(float(np.clip((np.trace(error[:3,:3])-1.)/2.,-1.,1.)))
            if (np.linalg.norm(error[:3,3])>self.grasp_position_tolerance or
                    angle>self.grasp_rotation_tolerance):
                raise ValueError('PREPARE_TOOL_COUPLING: joint path changes the captured two-hand grasp')
            samples.append(dict(t=float(fraction),q=dict(zip(self.model_names,q)),tool=tool))
        if self.collision_check is None:raise ValueError('PREPARE_TOOL_COLLISION_CHECK_REQUIRED')
        result=self.collision_check(samples)
        return dict(result,tool_present=True,coupling_checked=True)

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

    def independent_abi(self):
        """Resolve the additive capability only for ordinary-arm deployments."""
        p,i,d=c.c_void_p,c.c_int,c.c_double
        array=c.POINTER(d)
        try:
            for name,args in (
                ('bq_capture_independent',[p,array,i,c.POINTER(NativeConfig)]),
                ('bq_flange_poses',[p,array,i,array,array]),
                ('bq_solve_independent',[p,array,i,array,i,array,array,i,d,array,array])):
                fn=getattr(self.lib,name);fn.argtypes,fn.restype=args,i
        except AttributeError as exc:
            raise ValueError('INDEPENDENT_ARMS_ABI_MISSING: rebuild bindu_bimanual_qp') from exc

    def check_independent_path(self, feedback, *waypoints):
        if not getattr(self,'empty_collision_check',None):
            raise ValueError('INDEPENDENT_ARMS_COLLISION_CHECK_REQUIRED')
        before=np.array([feedback[n] for n in self.names]);samples=[]
        for end in waypoints:
            end=np.asarray(end,dtype=float)
            if end.shape!=before.shape or not np.isfinite(end).all():
                raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
            count=max(1,int(np.ceil(np.max(np.abs(end-before))/self.collision_step)))
            for fraction in np.linspace(0.,1.,count+1):
                q=dict(feedback);q.update(zip(self.names,before+fraction*(end-before)))
                samples.append(dict(t=float(len(samples)),q=q,tool=np.eye(4)))
            before=end
        self._validate_geometry(self.empty_collision_check,samples)

    def _apply_shared_motion_weight(self):
        if self.shared_motion_weight > 0 and not self.lib.bq_shared_motion_weight(self.handle,self.shared_motion_weight):
            raise ValueError('BIMANUAL_SHARED_MOTION_WEIGHT:'+self._error())

    def _check_measured_limits(self, measured):
        # Reference-based tasks must not hide actual joint/posture violations
        # when native kinematics are evaluated at the nominal configuration.
        for name, value in zip(self.names, measured):
            lo, hi = self.limits[name]
            if not lo-1e-9 <= value <= hi+1e-9:
                raise ValueError('BIMANUAL_MEASURED_POSITION_LIMIT:'+name)
        if self.posture_policy is not None and (np.any(measured < self.posture_lower-1e-9) or
                                                np.any(measured > self.posture_upper+1e-9)):
            raise ValueError('BIMANUAL_MEASURED_POSTURE_ERROR')

    def capture_independent(self, feedback, *, reference=None):
        self.captured=False;self.independent_abi()
        q=self._q(feedback);poses=np.empty((2,4,4),dtype=np.float64)
        measured=np.array([feedback[n] for n in self.names])
        if reference is not None:
            reference=np.asarray(reference,dtype=float)
            if reference.shape!=measured.shape or not np.isfinite(reference).all():
                raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
            if np.max(np.abs(reference-measured))>self.max_reference_error:
                raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
            self._check_measured_limits(measured)
            q=q.copy();q[[self.model_names.index(n) for n in self.names]]=reference
        if (not self.lib.bq_capture_independent(self.handle,self._ptr(q),self.nq,c.byref(self.native_config)) or
                not self.lib.bq_flange_poses(self.handle,self._ptr(q),self.nq,
                                             self._ptr(poses[0]),self._ptr(poses[1]))):
            raise ValueError('INDEPENDENT_ARMS_CAPTURE:'+self._error())
        self._apply_shared_motion_weight()
        self.check_independent_path(feedback,measured if reference is None else reference)
        self.grasp_mode='independent';self.captured=True
        return poses

    def solve_independent(self, feedback, targets, *, active_mask, reference, dt):
        """Two flange tasks with independent clutches; waist/inactive arms hold."""
        if not self.captured or self.grasp_mode!='independent':
            raise ValueError('INDEPENDENT_ARMS_NOT_CAPTURED')
        started=time.perf_counter();cpu_started=time.thread_time()
        self.last_geometry_step=self.last_timing=None
        try:
            if not math.isfinite(dt) or not 0<dt<=self.max_interval:
                raise ValueError('BIMANUAL_INTEGRATION_INTERVAL')
            if type(active_mask) is not int or not 1<=active_mask<=3:
                raise ValueError('INDEPENDENT_ARMS_CLUTCH_MASK')
            q=self._q(feedback)
            reference=np.ascontiguousarray(reference,dtype=np.float64)
            measured=np.array([feedback[n] for n in self.names])
            if reference.shape!=measured.shape or not np.isfinite(reference).all():
                raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
            if np.max(np.abs(reference-measured))>self.max_reference_error:
                raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
            if self.tracking_feedback == 'reference':
                self._check_measured_limits(measured)
                q = q.copy()
                q[[self.model_names.index(n) for n in self.names]] = reference
            poses=np.asarray(targets,dtype=float)
            if poses.shape!=(2,4,4):raise ValueError('INDEPENDENT_ARMS_TARGET_LAYOUT')
            left,right=(self._transform(pose) for pose in poses)
            out=np.empty(len(self.names));metrics=np.empty(3)
            native_at=time.perf_counter()
            if not self.lib.bq_solve_independent(self.handle,self._ptr(q),self.nq,
                    self._ptr(reference),len(self.names),self._ptr(left),self._ptr(right),active_mask,
                    dt,self._ptr(out),self._ptr(metrics)):
                raise ValueError('INDEPENDENT_ARMS_QP:'+self._error())
            native_us=(time.perf_counter()-native_at)*1e6
            if not np.isfinite(out).all() or not np.isfinite(metrics).all():
                raise ValueError('BIMANUAL_NONFINITE_RESULT')
            delta=out-reference;lead=reference-measured;moving=np.abs(delta)>1e-12
            reserve=.8*self.max_reference_error
            scale=float(np.clip(np.min((np.where(delta[moving]>0.,reserve,-reserve)-lead[moving])/
                                       delta[moving],initial=1.),0.,1.))
            out=reference+scale*delta;metrics[2]=scale
            for n,old,new in zip(self.names,reference,out):
                lo,hi=self.limits[n]
                if not lo<=new<=hi or abs(new-old)>self.speed[n]*dt+1e-6:
                    raise ValueError('BIMANUAL_POSITION_LIMIT:'+n)
            if np.max(np.abs(out-measured))>self.max_reference_error:
                raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
            geometry_at=time.perf_counter()
            out,geometry_scale=self.checked_geometry_step(feedback,reference,out,independent=True,
                deadline=started+self.cfg['solve_timeout_us']*1e-6)
            metrics[2]*=geometry_scale
            self._check_solve_budget('independent',started,cpu_started,native_us,metrics[0],
                                     (time.perf_counter()-geometry_at)*1e6)
            return tuple(float(v) for v in out),tuple(float(v) for v in metrics)
        except (ValueError,TypeError):
            self.captured=False
            raise

    def solve(self, feedback, target, *, reference, dt, target_stamp=None, tool_feedback=None):
        """Propose a step; caller commits reference/time only after acceptance.

        Optional reference kinematics integrate a coherent tool path at the
        accepted reference; measured feedback still gates grasp, tracking and
        collision safety. dt is explicit, not inferred from VR frequency. A large tracking error requires hold and a fresh capture;
        silently realigning a moving reference would introduce a position jump.
        tracking_feedback='reference' generates nominal motion without chasing
        measured compliance offsets. Measured joints/observations retain their
        safety checks; an optional tool observation replaces the tracking error
        only in the default 'measured' mode.
        Freshness and frame alignment are the caller's responsibility; explicit
        correction bounds are required and excessive disagreement rejects.
        """
        if not self.captured:
            raise ValueError('BIMANUAL_NOT_CAPTURED')
        self.last_timing = None
        self.last_geometry_step = None
        try:
            return self._solve(feedback, target, reference, float(dt), target_stamp, tool_feedback)
        except (ValueError, TypeError):
            self.captured = False
            raise

    def _solve(self, feedback, target, reference, dt, target_stamp, tool_feedback=None):
        started = time.perf_counter()
        cpu_started = time.thread_time()
        if not math.isfinite(dt) or not 0 < dt <= self.max_interval:
            raise ValueError('BIMANUAL_INTEGRATION_INTERVAL')
        q = self._q(feedback)
        reference = np.ascontiguousarray(reference, dtype=np.float64)
        if reference.shape != (len(self.names),) or not np.isfinite(reference).all():
            raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
        measured = np.array([feedback[n] for n in self.names])
        if self.posture_policy is not None and (np.any(measured<self.posture_lower-1e-9) or
                                                np.any(measured>self.posture_upper+1e-9)):
            raise ValueError('BIMANUAL_MEASURED_POSTURE_ERROR')
        if np.max(np.abs(reference-measured)) > self.max_reference_error:
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
        if self.tracking_feedback == 'reference':
            self._check_measured_limits(measured)
        measured_tool = None
        if self.reference_kinematics:
            measured_tool = np.empty((4,4), dtype=np.float64)
            if not self.lib.bq_tool_pose(self.handle,self._ptr(q),self.nq,self._ptr(measured_tool)):
                raise ValueError('BIMANUAL_MEASURED_TOOL:'+self._error())
            relative = np.empty((4,4),dtype=np.float64)
            if not self.lib.bq_relative_pose(self.handle,self._ptr(q),self.nq,self._ptr(relative)):
                raise ValueError('BIMANUAL_MEASURED_GRASP:'+self._error())
            if self.grasp_mode == 'rigid':
                error = np.linalg.inv(self.grasp_reference) @ relative
                angle = math.acos(float(np.clip((np.trace(error[:3,:3])-1.)/2.,-1.,1.)))
                distance = np.linalg.norm(error[:3,3])
            else:
                actual = np.linalg.inv(self.tool) @ relative @ self.right_grip
                distance = np.linalg.norm(actual[:3,3]-self.tool_grip[:3,3])
                angle = math.acos(float(np.clip(actual[:3,2] @ self.tool_grip[:3,2],-1.,1.)))
            if math.hypot(distance,angle) > self.cfg['max_relative_error']:
                raise ValueError('BIMANUAL_MEASURED_GRASP_ERROR')
            if self.expected_relative is not None:
                # Enforce the configured measured grip-point acceptance
                # separately from CAD contact tolerance and mixed QP residuals.
                point_error,angle=self._grip_error(relative)
                if (point_error>self.grasp_position_tolerance or
                        angle>self.grasp_rotation_tolerance):
                    raise ValueError('BIMANUAL_MEASURED_GRASP_ERROR')
            if tool_feedback is not None:
                observed = self._transform(tool_feedback)
                position_bound = float(self.cfg.get('max_tool_feedback_translation_m',0.))
                rotation_bound = float(self.cfg.get('max_tool_feedback_rotation_rad',0.))
                angle = math.acos(float(np.clip((np.trace(observed[:3,:3] @ measured_tool[:3,:3].T)-1.)/2.,-1.,1.)))
                if (not all(math.isfinite(x) and x > 0 for x in (position_bound,rotation_bound)) or
                        np.linalg.norm(observed[:3,3]-measured_tool[:3,3]) > position_bound or angle > rotation_bound):
                    raise ValueError('BIMANUAL_TOOL_FEEDBACK_DISAGREEMENT')
                measured_tool = np.ascontiguousarray(observed)
            # Measured non-controlled joints remain context; the integrated
            # controlled reference must have its own coherent FK/Jacobians.
            q = q.copy()
            q[[self.model_names.index(n) for n in self.names]] = reference
        elif tool_feedback is not None:
            raise ValueError('BIMANUAL_TOOL_FEEDBACK_REQUIRES_REFERENCE_KINEMATICS')
        target = np.ascontiguousarray(target, dtype=np.float64).reshape(4,4)
        out = np.empty(len(self.names), dtype=np.float64)
        metrics = np.empty(3, dtype=np.float64)
        if self.tracking:
            target = self._transform(target)
        args = (self.handle, self._ptr(q), self.nq, self._ptr(reference), len(self.names), self._ptr(target))
        if self.tracking:
            velocity = np.zeros(6, dtype=np.float64)
            if self.feedforward and target_stamp is not None:
                if not math.isfinite(target_stamp):
                    raise ValueError('BIMANUAL_TARGET_TIME')
                if self.target_sample is not None:
                    previous_stamp, previous = self.target_sample
                    gap = target_stamp-previous_stamp
                    if gap < 0 or (gap == 0 and not np.array_equal(target, previous)):
                        raise ValueError('BIMANUAL_TARGET_TIME')
                    if 0 < gap <= self.max_interval:
                        import pinocchio as pin
                        velocity[:3] = (target[:3,3]+target[:3,:3]@self.tracking_point-
                            previous[:3,3]-previous[:3,:3]@self.tracking_point)/gap
                        velocity[3:] = pin.log3(target[:3,:3] @ previous[:3,:3].T)/gap
            native_started = time.perf_counter()
            tracking_tool = measured_tool if self.tracking_feedback == 'measured' else None
            ok = self.lib.bq_solve_tracking(*args, self._ptr(velocity),
                None if tracking_tool is None else self._ptr(tracking_tool), dt, self._ptr(out), self._ptr(metrics))
        else:
            native_started = time.perf_counter()
            ok = self.lib.bq_solve(*args, dt, self._ptr(out), self._ptr(metrics))
        native_us = (time.perf_counter()-native_started)*1e6
        if not ok:
            raise ValueError('BIMANUAL_QP:'+self._error())
        self._check_solve_budget('native', started, cpu_started, native_us, metrics[0])
        if not np.isfinite(out).all() or not np.isfinite(metrics).all():
            raise ValueError('BIMANUAL_NONFINITE_RESULT')
        if self.reference_kinematics:
            # A position executor can lag under changing input cadence. Keep
            # the integrated target inside the measured tracking budget with
            # 20% transport reserve; shrink the whole coupled step uniformly.
            # The hard measured/reference guard above remains independent.
            delta = out-reference
            lead = reference-measured
            reserve = .8*self.max_reference_error
            moving = np.abs(delta)>1e-12
            bounds = np.where(delta[moving]>0.,reserve,-reserve)
            scale = float(np.clip(np.min((bounds-lead[moving])/delta[moving],initial=1.),0.,1.))
            out = reference+scale*delta
            metrics[2] *= scale
        if np.max(np.abs(out-measured)) > self.max_reference_error:
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
        for name, old, new in zip(self.names, reference, out):
            lo, hi = self.limits[name]
            if not lo <= new <= hi or abs(new-old) > self.speed[name]*dt+1e-6:
                raise ValueError('BIMANUAL_POSITION_LIMIT:'+name)
        if self.posture_policy is not None and (np.any(out<self.posture_lower-1e-9) or
                                                np.any(out>self.posture_upper+1e-9)):
            raise ValueError('BIMANUAL_POSTURE_LIMIT')
        geometry_started = time.perf_counter()
        # A synchronized position device follows measured -> candidate directly;
        # also check candidate -> prior reference to retain both endpoints.
        out,geometry_scale=self.checked_geometry_step(feedback,reference,out,
            deadline=started+self.cfg['solve_timeout_us']*1e-6)
        metrics[2]*=geometry_scale
        if any(not self.limits[n][0]<=v<=self.limits[n][1] for n,v in zip(self.names,out)):
            raise ValueError('BIMANUAL_POSITION_LIMIT:geometry_step')
        geometry_us = (time.perf_counter()-geometry_started)*1e6
        result = tuple(float(v) for v in out), tuple(float(v) for v in metrics)
        next_target_sample = None
        if self.feedforward and target_stamp is not None:
            next_target_sample = (target_stamp, target.copy())
        self._check_solve_budget('validation', started, cpu_started, native_us, metrics[0], geometry_us)
        if self.feedforward:
            self.target_sample = next_target_sample
        return result

    def _check_solve_budget(self, stage, started, cpu_started, native_us, qp_us, geometry_us=0.):
        # Read the deadline after building diagnostics/output, so a collection
        # during result packing cannot turn a late result into an accepted one.
        timing = dict(stage=stage, wall_us=0., thread_cpu_us=0., native_wall_us=native_us,
                      qp_internal_us=float(qp_us), geometry_wall_us=geometry_us)
        timing['thread_cpu_us'] = (time.thread_time()-cpu_started)*1e6
        timing['wall_us'] = (time.perf_counter()-started)*1e6
        self.last_timing = timing
        if timing['wall_us'] > float(self.cfg['solve_timeout_us']):
            raise ValueError(f"BIMANUAL_QP_TIMEOUT:{stage}:wall_us={timing['wall_us']:.0f}:"
                             f"cpu_us={timing['thread_cpu_us']:.0f}:native_us={native_us:.0f}:"
                             f"qp_us={timing['qp_internal_us']:.0f}:geometry_us={geometry_us:.0f}")
