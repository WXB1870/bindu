"""The supplied CAD export and its control reduction must stay aligned."""
from tests.support.paths import ROOT
import unittest
import hashlib
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from bindu_contracts.profile import Profile

DESCRIPTION=ROOT/'src/hardware/bindu_description'
CONFIG=ROOT/'src/integration/bindu_runtime/config'


class SnowbotDescription(unittest.TestCase):
    def test_full_model_meshes_and_control_joint_reduction(self):
        full=ET.parse(DESCRIPTION/'urdf/snowbot.urdf').getroot()
        display=ET.parse(DESCRIPTION/'urdf/snowbot_display.urdf').getroot()
        reduced=ET.parse(DESCRIPTION/'urdf/snowbot_control.urdf').getroot()
        self.assertEqual(len(full.findall('link')),33)
        self.assertEqual(len(display.findall('link')),33)
        self.assertEqual(len(reduced.findall('link')),33)
        names=[j.get('name') for j in full.findall('joint')]
        self.assertEqual(len(names),len(set(names)))
        for mesh in full.findall('.//mesh'):
            prefix='package://bindu_description/meshes/snowbot/'
            self.assertTrue(mesh.get('filename').startswith(prefix))
            self.assertTrue((DESCRIPTION/'meshes/snowbot'/mesh.get('filename')[len(prefix):]).is_file())
        self.assertFalse(reduced.findall('.//mesh'))
        self.assertEqual(len(display.findall('.//mesh')),len(full.findall('.//mesh')))
        moving={j.get('name') for j in reduced.findall('joint') if j.get('type')!='fixed'}
        profile=Profile.load(CONFIG/'robots/snowbot/snowbot_sim.json')
        self.assertEqual(moving,{n for names in profile.groups.values() for n in names})
        self.assertEqual(profile.groups['waist'],['body_joint_1','body_joint_2'])
        self.assertEqual(len(profile.groups['left_arm']),7)
        self.assertEqual(len(profile.groups['right_arm']),7)
        for j in reduced.findall('joint'):
            if j.get('name').startswith('car_wheel_'):
                self.assertEqual(j.get('type'),'fixed')
        for j in display.findall('joint'):
            if j.get('name').startswith('car_wheel_'):
                self.assertEqual(j.get('type'),'fixed')

    def test_all_variants_use_supplied_twenty_joint_model(self):
        from tools.models.prepare_snowbot_description import JOINT_SOURCE, JOINT_SOURCE_SHA256
        self.assertEqual(hashlib.sha256(JOINT_SOURCE.read_bytes()).hexdigest(),JOINT_SOURCE_SHA256)
        supplied={j.get('name'):j for j in ET.parse(JOINT_SOURCE).getroot().findall('joint')
                  if j.get('type')!='fixed'}
        self.assertEqual(len(supplied),20)
        for name in ('snowbot.urdf','snowbot_display.urdf','snowbot_control.urdf'):
            model=ET.parse(DESCRIPTION/'urdf'/name).getroot()
            for joint,source in supplied.items():
                actual=model.find(f"joint[@name='{joint}']")
                self.assertEqual(actual.get('type'),source.get('type'))
                for tag in ('parent','child','origin','axis','limit'):
                    self.assertEqual(actual.find(tag).attrib,source.find(tag).attrib,(name,joint,tag))
        # The virtual input world/radar scaffold is not a replacement for the
        # established physical-base and F/T/TCP interfaces or their meshes.
        full=ET.parse(DESCRIPTION/'urdf/snowbot.urdf').getroot()
        for link in ('car_base_link','arm_l_ft_sensor','arm_r_ft_sensor'):
            self.assertIsNotNone(full.find(f"link[@name='{link}']"))
        self.assertIsNone(full.find("link[@name='world']"))
        self.assertIsNone(full.find("link[@name='body_radar_link']"))

    def test_profiles_never_exceed_updated_model(self):
        model=ET.parse(DESCRIPTION/'urdf/snowbot_control.urdf').getroot()
        for name in ('snowbot_sim.json','snowbot_shovel_sim.json',
                     'snowbot_real_position.json','snowbot_vendor_position.json'):
            profile=Profile.load(CONFIG/'robots/snowbot'/name)
            for joint,(lo,hi) in profile.limits.items():
                limit=model.find(f"joint[@name='{joint}']/limit")
                self.assertGreaterEqual(lo,float(limit.get('lower')),(name,joint))
                self.assertLessEqual(hi,float(limit.get('upper')),(name,joint))
        limit=model.find("joint[@name='arm_r_joint_7']/limit")
        self.assertLess(1.42,float(limit.get('upper')))

    def test_real_and_simulation_selection_load_updated_models_without_ros(self):
        from bindu_runtime.robot_setup import RobotSetup
        packages={'bindu_description':DESCRIPTION,'bindu_runtime':CONFIG.parent}
        for filename in ('robot_snowbot.json','robot_snowbot_real.json'):
            selection=RobotSetup.load(CONFIG/'robots/snowbot'/filename,packages)
            self.assertEqual(selection.urdf,DESCRIPTION/'urdf/snowbot_control.urdf')
            self.assertEqual(selection.display_urdf,DESCRIPTION/'urdf/snowbot_display.urdf')
            self.assertLessEqual(selection.profile.limits['arm_r_joint_6'][1],1.274)
            self.assertLessEqual(selection.profile.limits['arm_r_joint_7'][1],1.449)

    def test_mesh_reuse_regeneration_reproduces_checked_in_variants(self):
        from tools.models.prepare_snowbot_description import apply_joint_source,write_variants
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)
            root=ET.parse(DESCRIPTION/'urdf/snowbot.urdf').getroot()
            # Reproduce stale CAD bounds; merely rewriting the checked-in
            # output would not prove the new source is actually applied.
            root.find("joint[@name='arm_r_joint_6']/limit").set('upper','1.57')
            root.find("joint[@name='arm_r_joint_7']/limit").set('lower','-1.57')
            root.find("joint[@name='head_joint_1']/limit").set('upper','3.14')
            write_variants(apply_joint_source(root),output)
            for name in ('snowbot.urdf','snowbot_display.urdf','snowbot_control.urdf'):
                self.assertEqual((output/'urdf'/name).read_bytes(),(DESCRIPTION/'urdf'/name).read_bytes())

    def test_changed_input_or_topology_cannot_silently_replace_model(self):
        from tools.models.prepare_snowbot_description import apply_joint_source
        model=ET.parse(DESCRIPTION/'urdf/snowbot.urdf').getroot()
        before=ET.tostring(model)
        model.find("joint[@name='arm_r_joint_7']/parent").set('link','wrong_link')
        changed=ET.tostring(model)
        with self.assertRaisesRegex(ValueError,'JOINT_SOURCE_TOPOLOGY:arm_r_joint_7'):
            apply_joint_source(model)
        self.assertEqual(ET.tostring(model),changed)
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'wrong.urdf';file.write_bytes(before)
            with self.assertRaisesRegex(ValueError,'JOINT_SOURCE_HASH_MISMATCH'):
                apply_joint_source(model,file)


if __name__=='__main__': unittest.main()
