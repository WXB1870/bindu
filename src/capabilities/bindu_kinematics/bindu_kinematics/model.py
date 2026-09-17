"""Named, fixed-base single-arm model. No geometry/collision claim is made.

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
        reference = pin.neutral(self.full)
        locked = []
        for jid in range(1, self.full.njoints):
            joint = self.full.joints[jid]
            name = self.full.names[jid]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError('MODEL_REQUIRES_SCALAR_JOINTS')
            if name not in self.names:
                reference[joint.idx_q] = config['locked_joints'][name]
                locked.append(jid)
        self.model = pin.buildReducedModel(self.full, locked, reference)
        self.data = self.model.createData()
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.names]
        self.lower = self.model.lowerPositionLimit[self.indices]
        self.upper = self.model.upperPositionLimit[self.indices]
        self.frame = self.model.getFrameId(config['ee_link'])
        if self.frame >= self.model.nframes:
            raise ValueError('MODEL_UNKNOWN_FRAME')

    def fk(self, values):
        q = np.zeros(self.model.nq)
        q[self.indices] = values
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.frame].homogeneous @ self.offset

    def symbolic_fk(self):
        import casadi as ca
        root = ET.parse(self.cfg['urdf']).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], self.cfg['ee_link']
        while link in by_child:
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        q = ca.SX.sym('q', len(self.names))
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
            value = q[self.names.index(name)] if name in self.names else self.cfg['locked_joints'][name]
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
        return q, transform @ ca.DM(self.offset)
