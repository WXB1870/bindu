"""Source geometry belongs to the package; generated physics must detect drift."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import pinocchio as pin

ROOT=Path(__file__).resolve().parents[1]
DESC=ROOT/'src/hardware/bindu_description'
sys.path.insert(0,str(ROOT/'tools'))
from prepare_g1_description import verify_description
from prepare_g1_physics import prepare, validate_physics_model


def structure(node):
    return (node.tag,tuple(sorted(node.attrib.items())),tuple(structure(n) for n in node if n.tag not in ('visual','collision')))


class G1AssetsTests(unittest.TestCase):
    def test_old_unregistered_assets_and_modified_meshes_are_rejected(self):
        manifest=verify_description(DESC)
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)
            for name in list(manifest['files'])+['g1.assets.json']:
                path=output/name;path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes((DESC/name).read_bytes())
            verify_description(output)
            stale=output/'meshes/g1/stale.stl';stale.write_bytes(b'old installation')
            with self.assertRaisesRegex(ValueError,'G1_ASSET_LAYOUT_MISMATCH'):
                verify_description(output)
            stale.unlink()
            mesh=output/next(name for name in manifest['files'] if name.startswith('meshes/'))
            mesh.write_bytes(mesh.read_bytes()+b'changed')
            with self.assertRaisesRegex(ValueError,'G1_ASSET_CHECKSUM_MISMATCH'):
                verify_description(output)

    def test_source_geometry_and_control_share_body_and_resolve_meshes(self):
        manifest=verify_description(DESC)
        self.assertEqual(manifest['counts'],{'meshes':78,'mesh_references':86,'source_usd_files':26})
        control=ET.parse(DESC/'urdf/g1_provisional.urdf').getroot()
        display=ET.parse(DESC/'urdf/g1_display.urdf').getroot()
        self.assertEqual(structure(control),structure(display))
        self.assertEqual(display.find("joint[@name='leg_joint4']").get('type'),'revolute')
        self.assertFalse(list(control.iter('mesh')))
        model=pin.buildModelFromUrdf(str(DESC/'urdf/g1_display.urdf'))
        self.assertEqual(model.nq,20)
        for kind in (pin.GeometryType.VISUAL,pin.GeometryType.COLLISION):
            geometry=pin.buildGeomFromUrdf(model,str(DESC/'urdf/g1_display.urdf'),kind,package_dirs=[str(DESC.parent)])
            self.assertTrue(geometry.geometryObjects)
            self.assertTrue(all(g.geometry is not None for g in geometry.geometryObjects))

    def test_physics_generation_uses_shared_assets_and_guards_outputs(self):
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'physics'
            prepare(None,output)
            record=validate_physics_model(output)
            self.assertFalse((output/'meshes').exists())
            self.assertFalse((output/'upstream_usd').exists())
            model=ET.parse(output/'g1_physics.urdf').getroot()
            self.assertEqual(model.find("joint[@name='leg_joint4']").get('type'),'revolute')
            for mesh in model.iter('mesh'):
                self.assertTrue(Path(mesh.get('filename')).is_relative_to(DESC))
                self.assertTrue(Path(mesh.get('filename')).is_file())
            profile=json.loads((output/'g1_physics_sim.json').read_text())
            self.assertIn('leg_joint4',profile['groups']['waist'])
            self.assertEqual(record['source_usd'],str(DESC/'usd/g1'))
            original=(output/'physics.json').read_bytes();(output/'physics.json').write_bytes(original+b'\n')
            with self.assertRaisesRegex(ValueError,'G1_PHYSICS_OUTPUT_CHANGED'):
                validate_physics_model(output)

    def test_profile_and_cache_schema_drift_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);profile=root/'src/integration/bindu_runtime/config/g1_provisional_sim.json'
            profile.parent.mkdir(parents=True);profile.write_bytes((ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json').read_bytes())
            output=root/'physics';prepare(None,output,root=root,description=DESC)
            validate_physics_model(output,root=root,description=DESC)
            profile.write_bytes(profile.read_bytes()+b'\n')
            with self.assertRaisesRegex(ValueError,'G1_PHYSICS_INPUTS_CHANGED'):
                validate_physics_model(output,root=root,description=DESC)
            record=json.loads((output/'provenance.json').read_text());record.pop('schema_version')
            (output/'provenance.json').write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError,'G1_PHYSICS_CACHE_LEGACY'):
                validate_physics_model(output,root=root,description=DESC)


if __name__=='__main__':unittest.main()
