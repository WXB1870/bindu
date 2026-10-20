"""Shared PICO button and feedback fixture."""
import numpy as np

from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from bindu_teleoperation.pico import PicoControl, PicoFeedback, PICO_MAPPING


class Rig:
    def __init__(self,load='empty',guided=False,dual=False):
        self.core=PicoControl(initial_load=load,guided_loading=guided,dual_modes=dual)
        self.now=1.;self.seq=0;self.source='headset-1'
        self.buttons=[0,0];self.triggers=[0.,0.];self.grips=[0.,0.]
        self.counters=[ButtonCounters(),ButtonCounters()]
        self.modes=('closed','closed');self.stopped=True
        self.step();self.step()

    def step(self,dt=.02):
        self.now+=dt;self.seq+=1
        self.core.update_feedback(PicoFeedback(self.now,self.stopped,self.modes),self.now)
        frames=[]
        for i,side in enumerate(('left','right')):
            pose=np.eye(4);pose[0,3]=i*.34
            frame=VRFrame(self.source,self.seq,self.now,side,tuple(pose.ravel()),self.grips[i],
                          self.triggers[i],mapping=PICO_MAPPING,buttons=self.buttons[i])
            frames.append(self.counters[i].update(frame))
        self.core.ingest(*frames,self.now)
        return self.core.drain()

    def click(self,side,bit):
        self.buttons[side]|=1<<bit;effects=self.step()
        self.buttons[side]&=~(1<<bit);effects+=self.step()
        return effects

    def complete(self,ok=True,reason=''):
        intent=self.core.pending
        self.now+=.01
        self.core.update_feedback(PicoFeedback(self.now,True,self.modes),self.now)
        self.core.complete(intent.token,ok,reason,self.now)
        return intent

    def context(self,load=None,supported=True):
        self.core.context(self.source,self.core.generation,load or self.core.load,supported,self.now)
        self.step()

    def loading(self):
        self.context()
        intents=self.click(0,2)
        assert intents[-1].operation=='stop'
        self.complete();self.click(1,1)
        assert self.core.phase=='loading'

    def held(self):
        self.context('held')
        self.click(1,1);self.complete();self.step()
        assert self.core.phase=='tool_standby'
