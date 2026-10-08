#!/usr/bin/env python3
"""Check planned joint and flange poses in Isaac Sim; no video or ROS commands.

Run with Isaac Sim's python.sh in an environment without ROS PYTHONPATH or
LD_LIBRARY_PATH. Joint positions are set with zero gravity: this validates the
URDF import and pose propagation, not a position servo, payload or contact.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET


def verify(motion_path, output):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Usd, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction

    motion = json.loads(motion_path.read_text())
    source = Path(motion['display_urdf'])
    if motion['schema'] != 1 or hashlib.sha256(source.read_bytes()).hexdigest() != motion['display_sha256']:
        raise ValueError('MOTION_MODEL_IDENTITY')
    if not motion['samples']:
        raise ValueError('EMPTY_MOTION')
    tree = ET.parse(source)
    package = source.parent.parent
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        prefix = f'package://{package.name}/'
        if uri.startswith(prefix):
            mesh.set('filename', str(package/uri.removeprefix(prefix)))
        elif not Path(uri).is_absolute():
            raise ValueError('UNRESOLVED_ISAAC_MESH:'+uri)
    roots = {link.get('name') for link in tree.findall('link')} - {
        joint.find('child').get('link') for joint in tree.findall('joint')}
    if len(roots) != 1:
        raise ValueError('URDF_REQUIRES_ONE_ROOT')
    absolute = output/'model.urdf'
    tree.write(absolute, encoding='utf-8', xml_declaration=True)
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    usd = URDFImporter(URDFImporterConfig(urdf_path=str(absolute), usd_path=str(output/'usd'),
                                        allow_self_collision=False)).import_urdf()
    world = World(stage_units_in_meters=1., physics_dt=.01, rendering_dt=.05)
    model = world.stage.DefinePrim('/World/Robot', 'Xform')
    model.GetReferences().AddReference(str(usd))
    links = {prim.GetName(): prim for prim in Usd.PrimRange(model)
             if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    root = roots.pop()
    frames = (motion['left_frame'], motion['right_frame'])
    if not {root, *frames}.issubset(links):
        raise ValueError('ISAAC_LINK_LAYOUT')
    body = world.scene.add(SingleArticulation(str(links[root].GetPath()), name='robot'))
    flanges = {name: world.scene.add(SingleRigidPrim(str(links[name].GetPath()), name=name))
               for name in frames}
    world.get_physics_context().set_gravity(0.)
    world.reset()
    names = body.dof_names
    maximum = dict(joint_rad=0., flange_translation_m=0., flange_rotation_rad=0., right_grip_m=0.)
    left_to_tool = np.array(motion['left_to_tool']).reshape(4, 4)
    grip_offset = np.r_[motion['recipe']['grip_offset_m'], 1.]
    right_in_tool = np.r_[motion['recipe']['right_grip_in_tool_m'], 1.]
    if not all(np.isfinite(value).all() for value in (left_to_tool, grip_offset, right_in_tool)):
        raise ValueError('NONFINITE_TOOL_GEOMETRY')
    for index, sample in enumerate(motion['samples']):
        if set(sample['q']) != set(names):
            raise ValueError('ISAAC_JOINT_LAYOUT')
        q = np.array([sample['q'][name] for name in names])
        if not np.isfinite(q).all():
            raise ValueError('NONFINITE_MOTION')
        body.set_joint_positions(q)
        body.set_joint_velocities(np.zeros_like(q))
        body.apply_action(ArticulationAction(joint_positions=q))
        # A joint-buffer readback alone does not propagate poses to links.
        world.step(render=index % 20 == 0)
        measured_q = body.get_joint_positions()
        if not np.isfinite(measured_q).all():
            raise ValueError('NONFINITE_ISAAC_JOINTS')
        maximum['joint_rad'] = max(maximum['joint_rad'], float(np.max(np.abs(measured_q-q))))
        actual = {}
        for name, link in flanges.items():
            position, wxyz = link.get_world_pose()
            expected = np.array(sample['flanges'][name])
            if (not np.isfinite(position).all() or not np.isfinite(wxyz).all() or
                    not np.isfinite(expected).all()):
                raise ValueError('NONFINITE_ISAAC_POSE')
            pose = np.eye(4)
            pose[:3, 3] = position
            pose[:3, :3] = Rotation.from_quat(np.r_[wxyz[1:], wxyz[0]]).as_matrix()
            actual[name] = pose
            maximum['flange_translation_m'] = max(maximum['flange_translation_m'],
                                                  float(np.linalg.norm(position-expected[:3, 3])))
            angle = Rotation.from_matrix(expected[:3, :3].T @ pose[:3, :3]).magnitude()
            maximum['flange_rotation_rad'] = max(maximum['flange_rotation_rad'], float(angle))
        actual_tool = actual[frames[0]] @ left_to_tool
        error = (actual[frames[1]] @ grip_offset - actual_tool @ right_in_tool)[:3]
        maximum['right_grip_m'] = max(maximum['right_grip_m'], float(np.linalg.norm(error)))
        if index % 100 == 0:
            print('POSE_CHECK', index, sample['t'], maximum, flush=True)
    if (maximum['joint_rad'] > .002 or maximum['flange_translation_m'] > .002 or
            maximum['flange_rotation_rad'] > .005 or maximum['right_grip_m'] > .002):
        raise ValueError('ISAAC_POSE_PROPAGATION:'+json.dumps(maximum))
    result = {'status': 'passed', 'samples': len(motion['samples']), 'maximum_errors': maximum,
              'motion_sha256': hashlib.sha256(motion_path.read_bytes()).hexdigest(),
              'motion': str(motion_path), 'mode': 'zero-gravity joint-position kinematic replay'}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise FileExistsError(output)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'fast_shutdown': True})
    try:
        verify(args.motion.resolve(), output)
    except BaseException:
        # Kit's atexit fast shutdown replaces uncaught exceptions with exit 0.
        # Preserve the diagnostic and failure status before that handler runs.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    app.close()


if __name__ == '__main__':
    main()
