"""Shared in-memory execution fixture; no scenario or test imports."""
import json
import tempfile
import unittest
from pathlib import Path

from bindu_contracts.profile import Profile
from bindu_contracts.contracts import Motion
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_hand import SimHandDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO
from bindu_execution.executor import Executor


class ExecutionFixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.profile_path = self.path/'profile.json'
        self.profile_path.write_text(json.dumps(dict(name='test',groups={'arm':['a','b'],'hand':['f']},
            limits={n:[-1,1] for n in ['a','b','f']},max_speed=1.5,
            capabilities=['joint_position','base_velocity','simulated_grasp'])))
        self.p = Profile.load(self.profile_path)
        self.drivers = {'arm': SimJointDriver(self.p.groups['arm'], self.p.max_speed),
                        'hand': SimHandDriver(self.p.groups['hand'], self.p.max_speed)}
        self.base = SimBaseDriver()
        self.io = RobotIO(self.p, self.drivers, self.base)
        self.e = Executor(self.p, self.io)
        lease, epoch = self.e.acquire('test',['arm','hand','base'],2.,10.)
        self.m = Motion('cmd',lease,epoch,self.p.digest,'task','obs','finite_trajectory','arm',10.,1.5,
                        names=('a','b'),offsets=(.5,),points=((.3,.3),))
