import math
import unittest
from bindu_hardware.robot_io.odometry import DifferentialOdometry


class WheelOdometryTests(unittest.TestCase):
    def test_encoder_straight_reverse_and_turn(self):
        odom=DifferentialOdometry(.1,.5)
        odom.update(2.,4.,10.)
        x,y,a,v,w=odom.update(3.,5.,11.)
        self.assertAlmostEqual(x,.1);self.assertAlmostEqual(y,0.)
        self.assertAlmostEqual(v,.1);self.assertAlmostEqual(w,0.)
        x,y,a,v,w=odom.update(2.,4.,12.)
        self.assertAlmostEqual(x,0.)
        x,y,a,v,w=odom.update(1.,5.,13.)
        self.assertAlmostEqual(x,0.);self.assertAlmostEqual(y,0.)
        self.assertAlmostEqual(a,.4);self.assertAlmostEqual(w,.4)

    def test_arc_uses_encoder_deltas_not_commands(self):
        odom=DifferentialOdometry(.1,.5);odom.update(0.,0.,0.)
        x,y,a,v,w=odom.update(1.,3.,2.)
        self.assertAlmostEqual(x,.5*math.sin(.4))
        self.assertAlmostEqual(y,.5*(1-math.cos(.4)))
        self.assertAlmostEqual(a,.4);self.assertAlmostEqual(v,.1)
        x2,y2,a2,v,w=odom.update(1.,3.,3.)
        self.assertEqual((x,y,a),(x2,y2,a2));self.assertEqual((v,w),(0.,0.))

    def test_bad_sample_does_not_change_state(self):
        for r,b in [(0.,1.),(1.,float('nan'))]:
            with self.assertRaises(ValueError):DifferentialOdometry(r,b)
        odom=DifferentialOdometry(.1,.5);odom.update(0.,0.,1.)
        for l,r,t in [(1.,1.,1.),(1.,1.,0.),(float('nan'),1.,2.)]:
            with self.assertRaises(ValueError):odom.update(l,r,t)
        self.assertAlmostEqual(odom.update(1.,1.,2.)[0],.1)

    def test_wrapped_encoder_angles_preserve_small_motion(self):
        odom=DifferentialOdometry(.1,.5);odom.update(math.pi-.02,-math.pi+.02,1.)
        x,y,a,v,w=odom.update(-math.pi+.03,math.pi-.03,1.1)
        self.assertAlmostEqual(x,0.);self.assertAlmostEqual(y,0.)
        self.assertAlmostEqual(a,-.02);self.assertAlmostEqual(w,-.2)
