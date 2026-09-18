"""Explicit Pi wire-to-robot mapping; never fill missing joints with zero."""
from collections import OrderedDict
import json
import math
import numpy as np

IMAGE_KEYS = ('left_wrist_rgb', 'right_wrist_rgb', 'front_rgb')


def load_config(path, profile):
    with open(path) as stream:
        cfg = json.load(stream)
    if cfg['mode'] not in ('pubsub', 'pull'):
        raise ValueError('PI_MODE')
    for key in ('state_bind', 'command_connect', 'state_topic', 'command_topic', 'request_topic'):
        if not isinstance(cfg[key], str) or not cfg[key]:
            raise ValueError('PI_CONFIG_' + key)
    mapping = cfg['joint_map']  # canonical robot name -> Pi wire name
    all_names = {j for names in profile.groups.values() for j in names}
    if (set(mapping) != all_names or len(set(mapping.values())) != len(mapping) or
            any(not isinstance(v, str) or not v for v in mapping.values())):
        raise ValueError('PI_JOINT_MAP')
    for key in ('command_max_age', 'observation_max_age', 'max_sync_skew', 'state_rate_hz', 'command_timeout'):
        if not isinstance(cfg[key], (float, int)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('PI_CONFIG_' + key)
    if not (cfg['command_max_age'] <= .5 and cfg['observation_max_age'] <= .5 and
            cfg['command_max_age'] <= cfg['command_timeout'] <= 2. and cfg['state_rate_hz'] <= 100):
        raise ValueError('PI_CONFIG_TIMING')
    if type(cfg['require_correlation']) is not bool:
        raise ValueError('PI_CONFIG_CORRELATION')
    groups = cfg.setdefault('resource_groups', list(profile.groups))
    if (not isinstance(groups, list) or not groups or any(not isinstance(g, str) for g in groups)
            or len(groups) != len(set(groups)) or not set(groups) <= set(profile.groups)):
        raise ValueError('PI_RESOURCE_GROUPS')
    return cfg


def observation_payload(cfg, positions, images, observation_id, session_id, prompt):
    mapping = cfg['joint_map']
    if not observation_id or set(positions) != set(mapping) or not all(math.isfinite(q) for q in positions.values()):
        raise ValueError('PI_OBSERVATION_JOINTS')
    if set(images) != set(IMAGE_KEYS):
        raise ValueError('PI_OBSERVATION_CAMERAS')
    for image in images.values():
        if image.dtype != np.uint8 or image.shape != (224, 224, 3):
            raise ValueError('PI_IMAGE_LAYOUT')
    return {'关节状态字典': {mapping[j]: float(q) for j, q in positions.items()},
            'RGB图像字典': images,
            # Optional extension: legacy model servers may ignore these fields.
            'bindu': {'session_id': session_id, 'observation_id': observation_id}, 'prompt': prompt}


class CommandAdapter:
    def __init__(self, cfg, profile, group, session_id, started_wall):
        if group not in cfg.get('resource_groups', profile.groups):
            raise ValueError('PI_RESOURCE_GROUPS')
        self.cfg, self.profile, self.group = cfg, profile, group
        self.session_id, self.started_wall = session_id, started_wall
        self.last_seq = -1
        self.last_stamp = 0.
        self.observations = OrderedDict()

    def observe(self, identity, stamp):
        self.observations[identity] = stamp
        while len(self.observations) > 128:
            self.observations.popitem(last=False)

    def convert(self, message, wall_now, ros_now):
        age = wall_now - message.timestamp_send
        if not 0 <= age <= self.cfg['command_max_age'] or message.timestamp_send < self.started_wall:
            raise ValueError('PI_STALE_OR_FUTURE_COMMAND')
        if message.seq <= self.last_seq or message.timestamp_send < self.last_stamp:
            raise ValueError('PI_REPLAYED_COMMAND')
        meta = message.payload.get('bindu')
        observation_id = ''
        if meta is not None:
            if not isinstance(meta, dict) or meta.get('session_id') != self.session_id:
                raise ValueError('PI_SESSION_MISMATCH')
            observation_id = meta.get('observation_id', '')
            source_stamp = self.observations.get(observation_id)
            if source_stamp is None or not 0 <= ros_now - source_stamp <= self.cfg['observation_max_age']:
                raise ValueError('PI_OBSERVATION_MISMATCH')
        elif self.cfg['require_correlation']:
            raise ValueError('PI_CORRELATION_REQUIRED')
        command = message.payload.get('关节命令字典', message.payload)
        names = tuple(self.profile.groups[self.group])
        try:
            values = tuple(command[self.cfg['joint_map'][j]] for j in names)
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError('PI_INVALID_POSITION')
            if any(not self.profile.limits[j][0] <= q <= self.profile.limits[j][1] for j, q in zip(names, values)):
                raise ValueError('PI_POSITION_LIMIT')
        except (KeyError, TypeError) as exc:
            raise ValueError('PI_MISSING_JOINT') from exc
        self.last_seq, self.last_stamp = message.seq, message.timestamp_send
        # Preserve source age; never restamp stale targets as newly generated.
        return names, tuple(float(v) for v in values), ros_now - age, observation_id
