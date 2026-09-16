"""Pi wire compatibility and action admission; no model or ROS required."""
from dataclasses import replace
import json
from pathlib import Path
import unittest
import numpy as np
from bindu_contracts.profile import Profile
from bindu_vla.pi.protocol import decode, encode, Message
from bindu_vla.pi.adapter import load_config, observation_payload, CommandAdapter, IMAGE_KEYS

ROOT = Path(__file__).resolve().parents[1]


class PiContracts(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/wheel_sim.json')
        self.cfg = load_config(ROOT/'src/integration/bindu_runtime/config/pi_loopback.json', self.profile)
        self.adapter = CommandAdapter(self.cfg, self.profile, 'arm', 'session', 10.)
        self.command = Message(self.cfg['command_topic'], 2, 10.1,
                               {'关节命令字典': {'left_joint_1': .2, 'left_joint_2': .3}}, {})

    def test_legacy_frames_and_numpy(self):
        # Fixed legacy-format frames, independent of our encoder.
        frames = [b'niic/pi/state', b'{"topic":"niic/pi/state","seq":3,"timestamp_send":10.0,"encoding":"json+numpy.raw","arrays":[{"dtype":"uint8","shape":[1,2,3]}]}',
                  b'{"image":{"__ndarray__":0}}', bytes(range(6))]
        message = decode(frames)
        np.testing.assert_array_equal(message.payload['image'], np.arange(6,dtype=np.uint8).reshape(1,2,3))
        encoded = encode('niic/pi/state',3,10.,message.payload)
        self.assertEqual(encoded[0], frames[0])
        self.assertEqual(json.loads(encoded[1]),json.loads(frames[1]))
        self.assertEqual(json.loads(encoded[2]),json.loads(frames[2]))
        self.assertEqual(encoded[3],frames[3])

    def test_malformed_frames(self):
        good = encode('x',0,10.,{'a':np.zeros((2,),dtype=np.uint8)})
        cases = [good[:2], good[:-1], good[:3]+[b'bad'], [b'x',b'{}',b'{}'],
                 [b'x',b'{"seq":1,"timestamp_send":0,"arrays":[{"dtype":"O","shape":[1]}]}',b'{}',bytes(8)],
                 [b'x',b'{"seq":1,"timestamp_send":0}',b'{"a":{"__ndarray__":-1}}']]
        for frames in cases:
            with self.subTest(frames=frames), self.assertRaises(ValueError): decode(frames)

    def test_snapshot_maps_real_fingers_without_synthetic_gripper(self):
        q = {j: .1 for names in self.profile.groups.values() for j in names}
        images = {name:np.zeros((224,224,3),dtype=np.uint8) for name in IMAGE_KEYS}
        payload=observation_payload(self.cfg,q,images,'obs','session','pick water')
        self.assertEqual(payload['关节状态字典']['left_finger_1'],.1)
        self.assertNotIn('left_hand',payload['关节状态字典'])
        self.assertEqual(payload['prompt'],'pick water')
        with self.assertRaises(ValueError): observation_payload(self.cfg,{},images,'obs','session','')
        with self.assertRaises(ValueError): observation_payload(self.cfg,q,{},'obs','session','')

    def test_legacy_target_preserves_age_and_no_false_correlation(self):
        names,values,stamp,obs = self.adapter.convert(self.command,10.2,50.)
        self.assertEqual(names,tuple(self.profile.groups['arm']))
        self.assertEqual(values,(.2,.3))
        self.assertAlmostEqual(stamp,49.9)
        self.assertEqual(obs,'')
        with self.assertRaisesRegex(ValueError,'REPLAYED'): self.adapter.convert(self.command,10.2,50.)

    def test_stale_future_before_session_and_invalid_positions(self):
        for message in (replace(self.command,timestamp_send=9.99),replace(self.command,timestamp_send=10.5),
                        replace(self.command,payload={'left_joint_1':.2}),
                        replace(self.command,payload={'left_joint_1':2.,'left_joint_2':.1}),
                        replace(self.command,payload={'left_joint_1':float('nan'),'left_joint_2':.1}),
                        replace(self.command,payload={'left_joint_1':[.2,.3],'left_joint_2':.1})):
            with self.subTest(message=message), self.assertRaises(ValueError): self.adapter.convert(message,10.2,50.)

    def test_optional_strict_identity_and_expired_observation(self):
        self.cfg['require_correlation']=True
        with self.assertRaisesRegex(ValueError,'CORRELATION_REQUIRED'): self.adapter.convert(self.command,10.2,50.)
        self.adapter.observe('obs',49.9)
        valid = replace(self.command,payload={**self.command.payload,'bindu':{'session_id':'session','observation_id':'obs'}})
        invalid=replace(valid,payload={**valid.payload,'bindu':{'session_id':'previous','observation_id':'obs'}})
        with self.assertRaisesRegex(ValueError,'SESSION_MISMATCH'): self.adapter.convert(invalid,10.2,50.)
        self.assertEqual(self.adapter.convert(valid,10.2,50.)[-1],'obs')
        with self.assertRaisesRegex(ValueError,'OBSERVATION_MISMATCH'):
            self.adapter.convert(replace(valid,seq=3,timestamp_send=10.6),10.7,50.5)


if __name__ == '__main__': unittest.main()
