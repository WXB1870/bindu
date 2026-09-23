import json
import math
from pathlib import Path
from .vr.mapping import pose_matrix, mapping_rotation


def load_config(path, profile, model_root):
    cfg = json.loads(Path(path).read_text())
    if cfg.get('simulation_only') is not True or cfg['side'] not in ('left', 'right'):
        raise ValueError('TELEOP_SIMULATION_CONFIG_REQUIRED')
    group = cfg['resource_group']
    if group not in profile.groups:
        raise ValueError('TELEOP_UNKNOWN_RESOURCE')
    model = cfg['kinematics']
    if 'collision_model' in model:
        collision_path = Path(model['collision_model'])
        if not collision_path.is_absolute():
            collision_path = Path(path).parent / collision_path
        model['collision'] = json.loads(collision_path.read_text())
    if tuple(model['joint_names']) != tuple(profile.groups[group]):
        raise ValueError('TELEOP_MODEL_LAYOUT')
    if model.get('measured_context', False):
        others = {n for names in profile.groups.values() for n in names}-set(model['joint_names'])
        if set(model['locked_joints']) != others:
            raise ValueError('TELEOP_CONTEXT_LAYOUT')
    for key in ('input_max_age', 'feedback_max_age', 'command_max_age', 'engage_seconds', 'position_scale'):
        if not isinstance(cfg[key], (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('TELEOP_INVALID_CONFIG: '+key)
    if cfg['input_max_age'] > cfg['command_max_age'] or cfg['command_max_age'] > 1.:
        raise ValueError('TELEOP_INVALID_FRESHNESS')
    # Preserve old configurations; the shipped example explicitly opts in.
    cfg.setdefault('mapping_mode', 'v34_anchor')
    cfg.setdefault('operator_yaw_rad', 0.)
    mapping_rotation(cfg['mapping_mode'], cfg['operator_yaw_rad'])
    for key in ('smooth_weight', 'solve_timeout', 'position_tolerance', 'rotation_tolerance', 'max_joint_step'):
        if not math.isfinite(model[key]) or model[key] <= 0:
            raise ValueError('TELEOP_INVALID_IK_CONFIG: '+key)
    pose_matrix(model['tool_transform'])
    path = Path(model['urdf'])
    model['urdf'] = str(path if path.is_absolute() else Path(model_root)/path)
    if not Path(model['urdf']).is_file():
        raise ValueError('TELEOP_MODEL_MISSING')
    return cfg
