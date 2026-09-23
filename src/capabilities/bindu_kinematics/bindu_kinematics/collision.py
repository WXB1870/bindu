"""Configured sphere/sphere and sphere/box self-collision proxy for local IK.

Frames come from the URDF, never robot-specific names in the solver. Numeric
checks use Pinocchio independently of the CasADi constraints. The sampled joint
segment is not a certificate for the downstream executor's continuous motion.
"""
import math
import numpy as np


class CollisionModel:
    def __init__(self, arm, config):
        self.arm = arm
        try:
            self.margin = float(config['margin'])
            self.step = float(config['joint_sample_step'])
            if not math.isfinite(self.margin) or self.margin <= 0 or not 0 < self.step <= .1:
                raise ValueError()
            self.shapes = config['shapes']
            if not isinstance(self.shapes, dict) or not self.shapes:
                raise ValueError()
            for shape in self.shapes.values():
                if arm.full.getFrameId(shape['link']) >= arm.full.nframes:
                    raise ValueError()
                center = np.asarray(shape['center'], dtype=float)
                if center.shape != (3,) or not np.isfinite(center).all():
                    raise ValueError()
                if shape['type'] == 'sphere':
                    if not math.isfinite(shape['radius']) or shape['radius'] <= 0:
                        raise ValueError()
                elif shape['type'] == 'box':
                    size = np.asarray(shape['half_extents'], dtype=float)
                    if size.shape != (3,) or not np.isfinite(size).all() or (size <= 0).any():
                        raise ValueError()
                else:
                    raise ValueError()
            self.pairs = []
            for pair in config['pairs']:
                if len(pair) != 2 or pair[0] == pair[1]:
                    raise ValueError()
                a, b = pair
                if self.shapes[a]['link'] == self.shapes[b]['link']:
                    raise ValueError()
                if self.shapes[a]['type'] != 'sphere':
                    a, b = b, a
                if self.shapes[a]['type'] != 'sphere':
                    raise ValueError()
                self.pairs.append((a, b))
            if not self.pairs or len(set(tuple(sorted(p)) for p in self.pairs)) != len(self.pairs):
                raise ValueError()
            self.links = tuple(sorted({s['link'] for s in self.shapes.values()}))
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError('IK_INVALID_COLLISION_CONFIG') from exc

    def symbolic(self, q, context):
        import casadi as ca
        transforms = {}
        for i, link in enumerate(self.links):
            x, c, matrix = self.arm.symbolic_fk(parameterize_context=True, link=link)
            transforms[link] = ca.Function('collision_frame_'+str(i), [x, c], [matrix])(q, context)
        constraints = []
        for a, b in self.pairs:
            sa, sb = self.shapes[a], self.shapes[b]
            ta, tb = transforms[sa['link']], transforms[sb['link']]
            pa = ta[:3, :3] @ ca.DM(sa['center']) + ta[:3, 3]
            pb = tb[:3, :3] @ ca.DM(sb['center']) + tb[:3, 3]
            radius = sa['radius'] + self.margin
            if sb['type'] == 'sphere':
                delta = pa-pb
                radius += sb['radius']
            else:
                local = tb[:3, :3].T @ (pa-pb)
                delta = ca.fmax(ca.fabs(local)-ca.DM(sb['half_extents']), 0)
            constraints.append(ca.sumsqr(delta)-radius**2)
        return ca.vertcat(*constraints)

    def clearances(self, q, context):
        transforms = self.arm.frames(q, context, self.links)
        out = {}
        for a, b in self.pairs:
            sa, sb = self.shapes[a], self.shapes[b]
            ta, tb = transforms[sa['link']], transforms[sb['link']]
            pa = ta[:3, :3] @ np.asarray(sa['center']) + ta[:3, 3]
            pb = tb[:3, :3] @ np.asarray(sb['center']) + tb[:3, 3]
            if sb['type'] == 'sphere':
                distance = np.linalg.norm(pa-pb)-sa['radius']-sb['radius']
            else:
                delta = np.abs(tb[:3, :3].T @ (pa-pb))-sb['half_extents']
                # Signed box distance also rejects centers inside the torso.
                distance = np.linalg.norm(np.maximum(delta, 0))+min(float(max(delta)), 0)-sa['radius']
            out[a+' / '+b] = float(distance)
        return out

    def safe(self, q, context):
        values = list(self.clearances(q, context).values())
        return bool(np.isfinite(values).all() and min(values) >= self.margin)

    def segment_safe(self, start, end, context):
        count = max(1, math.ceil(float(np.max(np.abs(end-start)))/self.step))
        if count > 256:
            return False
        return all(self.safe(start+(end-start)*i/count, context) for i in range(count+1))
