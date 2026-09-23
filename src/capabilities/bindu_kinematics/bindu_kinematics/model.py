"""Named, fixed-base arm and context FK, independent of collision geometry.

The numeric FK is Pinocchio. The CasADi graph is built from the same URDF chain
because binary Pinocchio distributions need not provide pinocchio.casadi.
"""
import xml.etree.ElementTree as ET
import numpy as np


class ArmModel:
    def __init__(self, config):
        import pinocchio as pin
        self.pin, self.cfg = pin, config
        self.names = tuple(config['joint_names'])
        self.offset = np.asarray(config['tool_transform'], dtype=float).reshape(4, 4)
        self.full = pin.buildModelFromUrdf(config['urdf'])
        if len(set(self.names)) != len(self.names):
            raise ValueError('MODEL_DUPLICATE_JOINT')
        if any(n not in self.full.names for n in self.names):
            raise ValueError('MODEL_UNKNOWN_JOINT')
        self.context_names = tuple(sorted(n for n in self.full.names[1:] if n not in self.names))
        self.context_default = tuple(config['locked_joints'][n] for n in self.context_names)
        self.context_indices = [self.full.joints[self.full.getJointId(n)].idx_q for n in self.context_names]
        self.full_indices = [self.full.joints[self.full.getJointId(n)].idx_q for n in self.names]
        self.full_data = self.full.createData()
        for jid in range(1, self.full.njoints):
            joint = self.full.joints[jid]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError('MODEL_REQUIRES_SCALAR_JOINTS')
        self.lower = self.full.lowerPositionLimit[self.full_indices]
        self.upper = self.full.upperPositionLimit[self.full_indices]
        if self.full.getFrameId(config['ee_link']) >= self.full.nframes:
            raise ValueError('MODEL_UNKNOWN_FRAME')

    def frames(self, values, context, links):
        q = self.pin.neutral(self.full)
        q[self.full_indices] = values
        q[self.context_indices] = self.context_default if context is None else context
        self.pin.framesForwardKinematics(self.full, self.full_data, q)
        return {link: self.full_data.oMf[self.full.getFrameId(link)].homogeneous.copy() for link in links}

    def fk(self, values, context=None):
        return self.frames(values, context, [self.cfg['ee_link']])[self.cfg['ee_link']] @ self.offset

    def symbolic_fk(self, parameterize_context=False, link=None):
        import casadi as ca
        root = ET.parse(self.cfg['urdf']).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        frame = link or self.cfg['ee_link']
        if self.full.getFrameId(frame) >= self.full.nframes:
            raise ValueError('MODEL_UNKNOWN_FRAME')
        chain, current = [], frame
        while current in by_child:
            j = by_child[current]
            chain.append(j)
            current = j.find('parent').get('link')
        q = ca.SX.sym('q', len(self.names))
        context = ca.SX.sym('context', len(self.context_names))
        transform = ca.SX.eye(4)
        for joint in reversed(chain):
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            fixed = np.eye(4)
            fixed[:3, :3] = self.pin.rpy.rpyToMatrix(*rpy)
            fixed[:3, 3] = xyz
            transform = transform @ ca.DM(fixed)
            kind, name = joint.get('type'), joint.get('name')
            if kind == 'fixed':
                continue
            value = (q[self.names.index(name)] if name in self.names else
                     context[self.context_names.index(name)] if parameterize_context else self.cfg['locked_joints'][name])
            axis_node = joint.find('axis')
            axis = np.asarray([float(x) for x in axis_node.get('xyz', '1 0 0').split()] if axis_node is not None else [1., 0., 0.])
            axis /= np.linalg.norm(axis)
            motion = ca.SX.eye(4)
            if kind in ('revolute', 'continuous'):
                skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
                motion[:3, :3] = ca.DM.eye(3) + ca.sin(value)*skew + (1-ca.cos(value))*(skew @ skew)
            elif kind == 'prismatic':
                motion[:3, 3] = value*axis
            else:
                raise ValueError('MODEL_UNSUPPORTED_JOINT')
            transform = transform @ motion
        result = transform if link is not None else transform @ ca.DM(self.offset)
        return (q, context, result) if parameterize_context else (q, result)
