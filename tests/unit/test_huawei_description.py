"""Legacy Huawei assets must remain coherent after moving out of the IK package."""
from tests.support.paths import ROOT
import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin

DESCRIPTION = ROOT/'src/hardware/bindu_description'


def kinematic_structure(node):
    return (node.tag,tuple(sorted(node.attrib.items())),
            tuple(kinematic_structure(n) for n in node if n.tag not in ('visual','collision')))


class HuaweiDescriptionTests(unittest.TestCase):
    def test_shared_meshes_match_source_aliases_and_hashes(self):
        manifest = json.loads((DESCRIPTION/'huawei_v34.assets.json').read_text())
        self.assertEqual(manifest['counts']['source_mesh_names'],101)
        self.assertEqual(manifest['counts']['stored_meshes'],94)
        self.assertEqual(manifest['counts']['deduplicated_aliases'],7)
        for name,entry in manifest['files'].items():
            data=(DESCRIPTION/name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(),entry['sha256'],name)
            self.assertEqual(len(data),entry['bytes'],name)
        for source,entry in manifest['source_meshes'].items():
            self.assertEqual(entry['sha256'],manifest['files'][entry['destination']]['sha256'],source)
        self.assertEqual(len({entry['sha256'] for name,entry in manifest['files'].items() if name.startswith('meshes/')}),94)
        display=ET.parse(DESCRIPTION/manifest['models']['display']).getroot()
        meshes={m.get('filename') for m in display.iter('mesh')}
        self.assertEqual(len(meshes),94)
        expected={'package://bindu_description/'+entry['destination'] for entry in manifest['source_meshes'].values()}
        self.assertEqual(meshes,expected)

    def test_control_and_display_preserve_the_same_robot_structure(self):
        control=ET.parse(DESCRIPTION/'urdf/huawei_v34_kinematic.urdf').getroot()
        display=ET.parse(DESCRIPTION/'urdf/huawei_v34_display.urdf').getroot()
        self.assertEqual(kinematic_structure(control),kinematic_structure(display))
        self.assertEqual(len(control.findall('link')),90)
        self.assertFalse(list(control.iter('mesh')))
        self.assertEqual(len(display.findall('.//visual')),88)
        self.assertEqual(len(display.findall('.//collision')),81)
        cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/huawei_v34/teleop_v34.json').read_text())['kinematics']
        moving={j.get('name') for j in control.findall('joint') if j.get('type')!='fixed'}
        self.assertEqual(moving,set(cfg['joint_names'])|set(cfg['locked_joints']))

    def test_packaged_meshes_load_for_both_geometry_roles(self):
        path=DESCRIPTION/'urdf/huawei_v34_display.urdf'
        model=pin.buildModelFromUrdf(str(path))
        self.assertEqual(model.nq,19)
        for kind,count in ((pin.GeometryType.VISUAL,88),(pin.GeometryType.COLLISION,81)):
            geometry=pin.buildGeomFromUrdf(model,str(path),kind,package_dirs=[str(DESCRIPTION.parent)])
            self.assertEqual(len(geometry.geometryObjects),count)
            self.assertTrue(all(obj.geometry is not None for obj in geometry.geometryObjects))
            data=model.createData();geometries=geometry.createData()
            pin.updateGeometryPlacements(model,data,geometry,geometries,pin.neutral(model))
            self.assertTrue(all(np.isfinite(pose.homogeneous).all() for pose in geometries.oMg))


if __name__=='__main__':unittest.main()
