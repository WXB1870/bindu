#!/usr/bin/env python3
"""Import the supplied Snowbot CAD export and derive its teleop kinematic model.

The original archive remains the source of record. This script makes only two
semantic changes: it repairs duplicated F/T joint names in the display model
and locks only the four free-spinning wheel joints in the controller model.
The distal eighth arm axes remain in the model and measured context but are
left to a future tool adapter; their grasp role is not specified by the URDF.
"""
import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile


SOURCE_SHA256 = '6d0786fe497d9021866b74e18cc64937fa3e6e31a7f8969efd208f013c788858'
SOURCE_PREFIX = 'snowbot_20260728_首次全身/snowbot/'
ACTIVE = {'body_joint_1', 'body_joint_2'} | {
    f'arm_{side}_joint_{axis}' for side in ('l', 'r') for axis in range(1, 8)
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'src/hardware/bindu_description')
    args = parser.parse_args()
    if hashlib.sha256(args.archive.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError('SNOWBOT_SOURCE_HASH_MISMATCH')
    with ZipFile(args.archive) as archive:
        members = {info.filename: info for info in archive.infolist()}
        if any((not name.startswith(SOURCE_PREFIX) and
                name not in (SOURCE_PREFIX.rstrip('/') + '/', SOURCE_PREFIX.split('/')[0] + '/')) or
               '..' in Path(name).parts
               for name in members):
            raise ValueError('SNOWBOT_ARCHIVE_PATH')
        source = archive.read(SOURCE_PREFIX + 'urdf/snowbot.urdf')
        root = ET.fromstring(source)
        joints = root.findall('joint')
        if (root.get('name') != 'snowbot' or len(root.findall('link')) != 33 or len(joints) != 32 or
                {j.get('name') for j in joints if j.get('type') != 'fixed'} !=
                ACTIVE | {'head_joint_1', 'head_joint_2',
                          'arm_l_joint_8', 'arm_r_joint_8'} |
                {f'car_wheel_{wheel}_joint' for wheel in ('fl', 'fr', 'bl', 'br')}):
            raise ValueError('SNOWBOT_UNEXPECTED_MODEL_LAYOUT')
        for side in ('l', 'r'):
            sensor = [j for j in joints if j.find('child').get('link') == f'arm_{side}_ft_sensor']
            if len(sensor) != 1 or sensor[0].get('name') != 'arm_r_joint_8':
                raise ValueError('SNOWBOT_FT_JOINT_LAYOUT')
            sensor[0].set('name', f'arm_{side}_ft_sensor_joint')
        names = [j.get('name') for j in joints]
        if len(names) != len(set(names)):
            raise ValueError('SNOWBOT_DUPLICATE_JOINT')
        meshes = set()
        for mesh in root.findall('.//mesh'):
            uri = mesh.get('filename')
            prefix = 'package://snowbot/meshes/'
            if not uri.startswith(prefix) or Path(uri[len(prefix):]).name != uri[len(prefix):]:
                raise ValueError('SNOWBOT_MESH_URI')
            meshes.add(uri[len(prefix):])
            mesh.set('filename', 'package://bindu_description/meshes/snowbot/' + uri[len(prefix):])
        output = args.output
        (output / 'urdf').mkdir(parents=True, exist_ok=True)
        mesh_dir = output / 'meshes/snowbot'
        mesh_dir.mkdir(parents=True, exist_ok=True)
        for name in sorted(meshes):
            (mesh_dir / name).write_bytes(archive.read(SOURCE_PREFIX + 'meshes/' + name))
        ET.indent(root, space='  ')
        ET.ElementTree(root).write(output / 'urdf/snowbot.urdf', encoding='unicode', xml_declaration=True)
        for joint in joints:
            if joint.get('name', '').startswith('car_wheel_'):
                joint.set('type', 'fixed')
                for tag in ('axis', 'limit', 'dynamics', 'safety_controller', 'mimic'):
                    element = joint.find(tag)
                    if element is not None:
                        joint.remove(element)
        ET.indent(root, space='  ')
        ET.ElementTree(root).write(output / 'urdf/snowbot_display.urdf',
                                   encoding='unicode', xml_declaration=True)
        for link in root.findall('link'):
            for tag in ('visual', 'collision'):
                for element in link.findall(tag):
                    link.remove(element)
        ET.indent(root, space='  ')
        ET.ElementTree(root).write(output / 'urdf/snowbot_control.urdf',
                                   encoding='unicode', xml_declaration=True)
    print(f'Imported {len(meshes)} meshes and three URDF models from {args.archive}')


if __name__ == '__main__':
    main()
