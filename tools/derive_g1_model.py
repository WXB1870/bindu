#!/usr/bin/env python3
"""Derive the provisional 19-axis kinematic model from a pinned upstream URDF."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

REVISION = '2d496b053f0d4e9e2688f59fac66022f447226be'
SOURCE_SHA256 = '8e7722191495d8b96ca8d64cc5dff918fb9cd673c74b70728641856d757e3b48'
GROUPS = {'leg': ['leg_joint1', 'leg_joint2'], 'waist': ['leg_joint3'],
          'head': ['head_joint1', 'head_joint2'],
          **{side+'_arm': [side+'_arm_joint'+str(i) for i in range(1, 8)]
             for side in ('left', 'right')}}


def derive(source, root):
    data = Path(source).read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise ValueError('G1_SOURCE_VERSION_MISMATCH')
    robot = ET.fromstring(data)
    robot.set('name', 'bindu_g1_provisional')
    # Wheels use the existing body v/omega adapter. No invented wheel geometry.
    removed = {link.get('name') for link in robot.findall('link')
               if link.get('name').startswith(('wheel_', 'left_gripper_', 'right_gripper_'))}
    active = [name for names in GROUPS.values() for name in names]
    for link in list(robot.findall('link')):
        if link.get('name') in removed:
            robot.remove(link)
        else:
            for child in list(link):
                if child.tag in ('visual', 'collision'):
                    link.remove(child)  # Kinematics only; meshes are not vendored.
    for joint in list(robot.findall('joint')):
        if joint.find('child').get('link') in removed or joint.find('parent').get('link') in removed:
            robot.remove(joint)
        elif joint.get('name') in ('leg_joint4', 'leg_joint5'):
            joint.set('type', 'fixed')  # Preserve the upstream zero-pose transform.
            for child in list(joint):
                if child.tag in ('axis', 'limit', 'dynamics'):
                    joint.remove(child)
    joints = {j.get('name'): j for j in robot.findall('joint') if j.get('type') != 'fixed'}
    if set(joints) != set(active):
        raise ValueError('G1_JOINT_LAYOUT_MISMATCH')
    profile = {'name': 'g1_provisional_sim', 'groups': GROUPS,
               'limits': {n: [float(joints[n].find('limit').get(k)) for k in ('lower', 'upper')] for n in active},
               'max_speed': .5, 'capabilities': ['joint_position', 'base_velocity'],
               'execution': {'max_acceleration': 2., 'max_jerk': 12., 'control_period': .01,
                             'max_tick_gap': .1, 'stop_timeout': 5., 'max_transition_duration': 30.,
                             'joint_dynamics': {n: {'velocity': min(.5, float(joints[n].find('limit').get('velocity')))} for n in active},
                             'base_limits': {'linear_velocity': .4, 'angular_velocity': .4,
                                             'linear_acceleration': 1., 'angular_acceleration': 2.}}}
    out = root/'src/hardware/bindu_description/urdf/g1_provisional.urdf'
    out.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot, space='  ')
    notice = ('<?xml version="1.0"?>\n<!-- Derived from GalaxyGeneralRobotics/galbot_one_golf_description\n'
              f'     commit {REVISION}; Apache-2.0, see ../LICENSE.\n'
              '     Bindu modifications: leg_joint4/5 fixed at zero; wheels and grippers omitted;\n'
              '     visual/collision meshes removed. Kinematics only, not calibrated hardware.\n'
              '     Regenerate with tools/derive_g1_model.py; do not hand-edit. -->\n')
    out.write_text(notice+ET.tostring(robot, encoding='unicode')+'\n')
    (root/'src/integration/bindu_runtime/config/g1_provisional_sim.json').write_text(json.dumps(profile, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    derive(args.source, Path(__file__).resolve().parents[1])
