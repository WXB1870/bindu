#!/usr/bin/env python3
"""Generate local G1 physics assets from the control model and pinned upstream geometry.

The output defaults to artifacts/g1-physics-model/ (ignored by Git). It contains
g1_physics.urdf, the simulation profile and provenance. Meshes and source USD
are read from bindu_description; they are not copied into generated caches.
The differential wheels/supports are provisional simulation fixtures. Regenerate
after changing g1_provisional.urdf, its profile or physics.json.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from tools.models.derive_g1_model import REVISION, SOURCE_SHA256
from tools.models.prepare_g1_description import (PREFIX, MANIFEST, digest, verify_description,
                                    prepare as prepare_description)

from tools import ROOT
DESCRIPTION = ROOT/'src/hardware/bindu_description'


def add_body(robot, name, mass, radius, width=None):
    link = ET.SubElement(robot, 'link', name=name)
    inertia = ET.SubElement(link, 'inertial')
    ET.SubElement(inertia, 'mass', value=str(mass))
    axial = mass * radius**2 / 2 if width else .4 * mass * radius**2
    radial = mass * (3 * radius**2 + width**2) / 12 if width else axial
    ET.SubElement(inertia, 'inertia', ixx=str(radial), iyy=str(axial), izz=str(radial),
                  ixy='0', ixz='0', iyz='0')
    for kind in ('visual', 'collision'):
        shape = ET.SubElement(link, kind)
        if width:
            ET.SubElement(shape, 'origin', rpy='1.5707963267948966 0 0')
        geometry = ET.SubElement(shape, 'geometry')
        ET.SubElement(geometry, 'cylinder' if width else 'sphere',
                      **({'radius': str(radius), 'length': str(width)} if width else {'radius': str(radius)}))
        if kind == 'visual':
            material = ET.SubElement(shape, 'material', name='simulation_fixture')
            ET.SubElement(material, 'color', rgba='0.08 0.12 0.16 1')


def inputs(description, root=ROOT):
    return {name:digest(path) for name,path in {
        'control':description/'urdf/g1_provisional.urdf',
        'display':description/'urdf/g1_display.urdf',
        'asset_manifest':description/MANIFEST,
        'physics':description/'physics.json',
        'profile':root/'src/integration/bindu_runtime/config/robots/g1/g1_provisional_sim.json',
        'preparation_recipe':Path(__file__),
    }.items()}


def validate_physics_model(model, root=ROOT, description=None):
    """Reject stale source geometry, body axes, profiles or prepared outputs."""
    model=Path(model);description=Path(description or root/'src/hardware/bindu_description').resolve()
    path=model/'provenance.json'
    if not path.is_file():raise ValueError('G1_PHYSICS_CACHE_MISSING: run tools/models/prepare_g1_physics.py')
    record=json.loads(path.read_text())
    if record.get('schema_version')!=2:
        raise ValueError('G1_PHYSICS_CACHE_LEGACY: regenerate with tools/models/prepare_g1_physics.py')
    if record.get('description')!=str(description):
        raise ValueError('G1_PHYSICS_ASSET_LOCATION_CHANGED: regenerate the cache')
    assets=verify_description(description)
    if record.get('source_usd')!=str(description/assets['source_usd']):
        raise ValueError('G1_PHYSICS_USD_SOURCE_CHANGED')
    if set(record.get('outputs',{}))!={'g1_physics.urdf','g1_physics_sim.json','physics.json'}:
        raise ValueError('G1_PHYSICS_CACHE_LAYOUT')
    if record.get('inputs')!=inputs(description,root):
        raise ValueError('G1_PHYSICS_INPUTS_CHANGED: regenerate the cache')
    for name,sha in record['outputs'].items():
        if Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError('G1_PHYSICS_OUTPUT_PATH')
        if not (model/name).is_file() or digest(model/name)!=sha:
            raise ValueError('G1_PHYSICS_OUTPUT_CHANGED: '+name)
    return record


def prepare(source, output, root=ROOT, description=None):
    description=Path(description or root/'src/hardware/bindu_description').resolve()
    if source is not None:
        prepare_description(Path(source),description)
    assets=verify_description(description)
    robot=ET.parse(description/'urdf/g1_display.urdf').getroot()
    robot.set('name','bindu_g1_physics')
    cfg=json.loads((description/'physics.json').read_text())
    resources={}
    for mesh in robot.iter('mesh'):
        uri=mesh.get('filename')
        if not uri.startswith(PREFIX):raise ValueError('G1_PHYSICS_MESH_URI')
        relative=uri.removeprefix(PREFIX)
        if relative not in assets['files']:raise ValueError('G1_PHYSICS_MESH_NOT_REGISTERED')
        # Isaac's standalone importer uses file paths without ROS package lookup.
        # Absolute references are local generated data; source URDFs keep package URIs.
        mesh.set('filename',str(description/relative))
        resources[relative]=assets['files'][relative]['sha256']
    output.mkdir(parents=True,exist_ok=True)
    for side, sign in [('left', 1), ('right', -1)]:
        name = f'sim_{side}_wheel'
        add_body(robot, name, cfg['wheel_mass'], cfg['wheel_radius'], cfg['wheel_width'])
        joint = ET.SubElement(robot, 'joint', name=name+'_joint', type='continuous')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=f"{cfg['wheel_center_x']} {sign*cfg['wheel_separation']/2} {cfg['wheel_center_z']}")
        ET.SubElement(joint, 'axis', xyz='0 1 0')
        ET.SubElement(joint, 'limit', effort=str(cfg['wheel_effort']), velocity='12')
    for i, x in enumerate(cfg['support_x']):
        name = f'sim_support_{i}'
        add_body(robot, name, .2, cfg['support_radius'])
        joint = ET.SubElement(robot, 'joint', name=name+'_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=name)
        z = cfg['wheel_center_z'] - cfg['wheel_radius'] + cfg['support_radius']
        ET.SubElement(joint, 'origin', xyz=f'{x} 0 {z}')
    ET.indent(robot, space='  ')
    path = output / 'g1_physics.urdf'
    path.write_text('<?xml version="1.0"?>\n<!-- Generated physics fixture; synthetic differential wheels/supports. Not calibrated hardware. -->\n'+ET.tostring(robot, encoding='unicode')+'\n')
    shutil.copyfile(description/'LICENSE',output/'LICENSE')
    shutil.copyfile(description/'physics.json',output/'physics.json')
    profile = json.loads((root / 'src/integration/bindu_runtime/config/robots/g1/g1_provisional_sim.json').read_text())
    profile['name'] = 'g1_physics_sim'
    # Simulation-only overrides stay with the body configuration. Public
    # execution math and the provisional kinematic profile remain unchanged.
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for group, dynamics in cfg.get('execution_group_dynamics', {}).items():
        if (group not in profile['groups'] or not dynamics or
                set(dynamics)-{'velocity','acceleration','jerk'} or
                any(isinstance(v, bool) or not isinstance(v, (int,float)) or
                    not math.isfinite(v) or v <= 0 for v in dynamics.values())):
            raise ValueError('INVALID_PHYSICS_EXECUTION_DYNAMICS')
        for name in profile['groups'][group]:
            if dynamics.get('velocity',0.) > float(joints[name].find('limit').get('velocity')):
                raise ValueError('PHYSICS_VELOCITY_EXCEEDS_URDF: '+name)
            profile['execution']['joint_dynamics'].setdefault(name, {}).update(dynamics)
    profile['execution']['feedback_tolerances'] = {
        'joint_position': .002, 'joint_velocity': .01,
        'base_linear_velocity': .003, 'base_angular_velocity': .005}
    (output / 'g1_physics_sim.json').write_text(json.dumps(profile, indent=2)+'\n')
    outputs={name:digest(output/name) for name in ('g1_physics.urdf','g1_physics_sim.json','physics.json')}
    (output/'provenance.json').write_text(json.dumps({'schema_version':2,
        'revision':REVISION,'urdf_sha256':SOURCE_SHA256,'description':str(description),
        'source_usd':str(description/assets['source_usd']),
        'generated_urdf_sha256':outputs['g1_physics.urdf'],'meshes':resources,
        'inputs':inputs(description,root),'outputs':outputs,
        'assumptions':cfg,'license':'Apache-2.0'},indent=2)+'\n')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, help='Optional pinned upstream import; normally use the packaged assets')
    parser.add_argument('--description',type=Path,default=DESCRIPTION,help='Shared bindu_description asset directory')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/g1-physics-model',
                        help='Generated asset directory (default: artifacts/g1-physics-model)')
    args = parser.parse_args()
    print(prepare(args.source,args.output.resolve(),description=args.description))


if __name__ == '__main__':
    main()
