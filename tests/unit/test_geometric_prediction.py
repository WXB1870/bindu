"""Geometry/box feasibility, independent-mode bypass and finite-stop regression."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from tools.analysis.replay_geometric_prediction import build, eigen_include, model_header

URDF='''<robot name="closed_pair">
<link name="base"/><link name="shared"/><link name="left"/><link name="right"/>
<link name="left_tip"/><link name="right_tip"/>
<joint name="common" type="prismatic"><parent link="base"/><child link="shared"/><axis xyz="1 0 0"/></joint>
<joint name="left_angle" type="revolute"><parent link="shared"/><child link="left"/><origin xyz="0 .2 0"/><axis xyz="0 0 1"/></joint>
<joint name="right_angle" type="revolute"><parent link="shared"/><child link="right"/><origin xyz="0 -.2 0"/><axis xyz="0 0 1"/></joint>
<joint name="left_fixed" type="fixed"><parent link="left"/><child link="left_tip"/><origin xyz=".3 0 0"/><axis xyz="0 0 0"/></joint>
<joint name="right_fixed" type="fixed"><parent link="right"/><child link="right_tip"/><origin xyz=".3 0 0"/></joint>
</robot>'''
NAMES=['common','left_angle','right_angle']
PAYLOAD='''3 3
-2 2 1 4.5 162
-2 2 1 4.5 162
-2 2 1 4.5 162
0 0 0
0 0 0 0 0
.05 .05 .03 0 0
.1 .1 .06 0 0
0 0 0
'''

class GeometricPredictionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not eigen_include() or not (shutil.which('clang++') or shutil.which('g++')):
            raise unittest.SkipTest('Existing C++17 compiler/Eigen required')
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.urdf=cls.root/'pair.urdf';cls.urdf.write_text(URDF)
        cls.exe=build(cls.root/'build',cls.urdf,NAMES,'left_tip','right_tip')

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def run_mode(self,mode,geometry=True):
        return subprocess.run([str(self.exe),mode,str(int(geometry))],input=PAYLOAD,text=True,capture_output=True,check=True)

    def test_analytic_jacobian_projection_retraction_and_infeasible_box(self):
        self.assertIn('CHECKS_OK',self.run_mode('check').stdout)

    def test_independent_arm_bypasses_geometry_exactly(self):
        self.assertEqual(self.run_mode('A',False).stdout,self.run_mode('geometry',False).stdout)

    def test_common_motion_survives_with_continuous_replacements_and_finite_stop(self):
        result=self.run_mode('geometry')
        events=[list(map(float,line.split()[1:])) for line in result.stderr.splitlines() if line.startswith('E ')]
        self.assertGreater(sum(x[3] for x in events),0)
        self.assertLess(max(x[2] for x in events),1e-8)
        rows=[list(map(float,line.split())) for line in result.stdout.splitlines()]
        self.assertGreater(max(x[1] for x in rows),.001)
        self.assertTrue(all(abs(x)<1e-8 for x in rows[-1][4:10]))

    def test_geometry_rejection_falls_back_without_unconstrained_prediction(self):
        payload=PAYLOAD.replace('0 0 0\n0 0 0 0 0\n', '0 .5 -.5\n0 0 0 .5 -.5\n')
        payload=payload.replace('.03 0 0', '.03 .5 -.5').replace('.06 0 0', '.06 .5 -.5')
        rejected=subprocess.run([str(self.exe),'geometry','1'],input=payload,text=True,capture_output=True,check=True)
        no_trend=payload.replace('.05 .05', '.05 0').replace('.1 .1', '.1 0')
        expected=subprocess.run([str(self.exe),'A','1'],input=no_trend,text=True,capture_output=True,check=True)
        events=[list(map(float,line.split()[1:])) for line in rejected.stderr.splitlines() if line.startswith('E ')]
        self.assertEqual(sum(x[3] for x in events),0)
        self.assertGreater(sum(x[6] for x in events),0)
        self.assertEqual(rejected.stdout,expected.stdout)

    def test_missing_moving_ancestor_rejected(self):
        with self.assertRaisesRegex(ValueError,'moving ancestor'):
            model_header(self.urdf,NAMES[1:],'left_tip','right_tip')

    def test_duplicate_names_rejected(self):
        with self.assertRaisesRegex(ValueError,'unique'):
            model_header(self.urdf,['common','common'],'left_tip','right_tip')

    def test_unknown_frame_rejected(self):
        with self.assertRaisesRegex(ValueError,'unknown end frame'):
            model_header(self.urdf,NAMES,'missing','right_tip')

    def test_fixed_joint_cannot_be_active(self):
        with self.assertRaisesRegex(ValueError,'unknown or fixed'):
            model_header(self.urdf,[*NAMES,'left_fixed'],'left_tip','right_tip')

    def test_time_order_violation_rejected(self):
        bad=PAYLOAD.replace('.1 .1 .06','-.1 .1 .06')
        result=subprocess.run([str(self.exe),'geometry','1'],input=bad,text=True,capture_output=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('INPUT_TIME_ORDER',result.stderr)

if __name__=='__main__':unittest.main()
