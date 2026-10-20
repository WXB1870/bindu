"""The supplied CAD export and its control reduction must stay aligned."""
from tests.support.paths import ROOT
import unittest
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


if __name__=='__main__': unittest.main()
