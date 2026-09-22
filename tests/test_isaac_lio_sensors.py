import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from isaac_lio_sensors import InertialSampler, rotation_xyzw


class InertialSamplerTests(unittest.TestCase):
    def test_stationary_tilt_specific_force(self):
        angle=.3
        rotation=rotation_xyzw([math.sin(angle/2),0.,0.,math.cos(angle/2)])
        sampler=InertialSampler()
        for t in (0.,.01,.02):
            a,w=sampler.sample(t,np.array([1.,2.,3.]),rotation)
            np.testing.assert_allclose(rotation @ a,[0.,0.,9.81],atol=1e-10)
            np.testing.assert_allclose(w,[0.,0.,0.],atol=1e-10)

    def test_translation_and_rotation_use_acquisition_time(self):
        sampler=InertialSampler()
        for t in np.arange(101)*.01:
            rotation=rotation_xyzw([0.,0.,math.sin(.2*t/2),math.cos(.2*t/2)])
            a,w=sampler.sample(float(t),np.array([.5*.4*t*t,0.,0.]),rotation)
        np.testing.assert_allclose(rotation @ a,[.4,0.,9.81],atol=1e-9)
        self.assertAlmostEqual(w[2],.2,places=6)
        with self.assertRaises(ValueError):sampler.sample(1.,np.zeros(3),np.eye(3))
