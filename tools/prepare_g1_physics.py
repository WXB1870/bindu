#!/usr/bin/env python3
"""Restore pinned G1 body geometry and add an explicitly provisional differential base."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import urllib.request
import xml.etree.ElementTree as ET

from derive_g1_model import REVISION, SOURCE_SHA256

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_SHA256 = 'efa6ff84b0393f680915a42f667f02afe8c0303d4edea9d4955dd6bdcf4b5fef'


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


def prepare(source, output, root=ROOT):
    data = (source / 'urdf/galbot_one_golf.urdf').read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise ValueError('G1_SOURCE_VERSION_MISMATCH')
    upstream = {link.get('name'): link for link in ET.fromstring(data).findall('link')}
    robot = ET.parse(root / 'src/hardware/bindu_description/urdf/g1_provisional.urdf').getroot()
    robot.set('name', 'bindu_g1_physics')
    cfg = json.loads((root / 'src/hardware/bindu_description/physics.json').read_text())
    output.mkdir(parents=True, exist_ok=True)
    resources = {}
    for link in robot.findall('link'):
        for original in upstream[link.get('name')]:
            if original.tag not in ('visual', 'collision'):
                continue
            element = copy.deepcopy(original)
            for mesh in element.iter('mesh'):
                relative = Path(mesh.get('filename'))
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError('INVALID_MESH_PATH')
                src = source / 'urdf' / relative
                dest = output / relative
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(src, dest)
                resources[str(relative)] = hashlib.sha256(src.read_bytes()).hexdigest()
            link.append(element)
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
    shutil.copyfile(source / 'LICENSE', output / 'LICENSE')
    # The local Isaac 6.0 RC importer leaves GLB visuals empty. Reuse the matching
    # upstream USD visual payloads (including textures) instead of inventing meshes.
    shutil.copytree(source / 'usd', output / 'upstream_usd', dirs_exist_ok=True)
    shutil.copyfile(root / 'src/hardware/bindu_description/physics.json', output / 'physics.json')
    profile = json.loads((root / 'src/integration/bindu_runtime/config/g1_provisional_sim.json').read_text())
    profile['name'] = 'g1_physics_sim'
    profile['execution']['feedback_tolerances'] = {
        'joint_position': .002, 'joint_velocity': .01,
        'base_linear_velocity': .003, 'base_angular_velocity': .005}
    (output / 'g1_physics_sim.json').write_text(json.dumps(profile, indent=2)+'\n')
    (output / 'provenance.json').write_text(json.dumps({'revision': REVISION, 'urdf_sha256': SOURCE_SHA256,
        'generated_urdf_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'meshes': resources,
        'assumptions': cfg, 'license': 'Apache-2.0'}, indent=2)+'\n')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, help='Existing pinned upstream repository')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/g1-physics-model')
    args = parser.parse_args()
    source = args.source
    if source is None:
        cache = args.output / 'upstream'
        cache.mkdir(parents=True, exist_ok=True)
        archive = cache / 'source.tar.gz'
        if not archive.exists():
            urllib.request.urlretrieve(f'https://codeload.github.com/GalaxyGeneralRobotics/galbot_one_golf_description/tar.gz/{REVISION}', archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
            raise ValueError('G1_ARCHIVE_CHECKSUM_MISMATCH')
        with tarfile.open(archive) as tar:
            tar.extractall(cache, filter='data')
        source = cache / f'galbot_one_golf_description-{REVISION}'
    print(prepare(source.resolve(), args.output.resolve()))


if __name__ == '__main__':
    main()
