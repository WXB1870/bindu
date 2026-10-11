"""One robot selection, checked against its model before launching any node.

No ROS imports or device access during validation. Relative paths belong to
the referencing file; package:// references use the installed ament index.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from bindu_contracts.profile import Profile
from bindu_teleoperation.config import load_config


def resolve_file(value, owner, packages=None):
    if not isinstance(value, str) or not value:
        raise ValueError('ROBOT_CONFIG_PATH')
    if value.startswith('package://'):
        package, sep, relative = value[10:].partition('/')
        if not package or not sep or '..' in Path(relative).parts:
            raise ValueError('ROBOT_CONFIG_PACKAGE_PATH')
        if packages is None:
            from ament_index_python.packages import get_package_share_directory
            base = Path(get_package_share_directory(package))
        else:
            base = Path(packages[package])
        path = base / relative
    else:
        path = Path(owner).parent / value
    path = path.resolve()
    if not path.is_file():
        raise ValueError('ROBOT_CONFIG_MISSING:' + str(path))
    return path


def model_layout(path):
    root = ET.parse(path).getroot()
    links = [link.get('name') for link in root.findall('link')]
    joints = root.findall('joint')
    names = [joint.get('name') for joint in joints]
    if (not links or None in links or len(set(links)) != len(links) or
            None in names or len(set(names)) != len(names)):
        raise ValueError('ROBOT_MODEL_NAMES')
    parents, bounds = {}, {}
    for joint in joints:
        name, kind = joint.get('name'), joint.get('type')
        parent, child = joint.find('parent'), joint.find('child')
        if (parent is None or child is None or parent.get('link') not in links or
                child.get('link') not in links or child.get('link') in parents):
            raise ValueError('ROBOT_MODEL_TREE:' + name)
        parents[child.get('link')] = parent.get('link')
        if joint.find('mimic') is not None:
            raise ValueError('ROBOT_MODEL_MIMIC_UNSUPPORTED:' + name)
        if kind == 'fixed':
            continue
        # Current Pinocchio/C ABI requires scalar q/v; reject rather than
        # silently counting continuous/floating/coupled joints as one axis.
        if kind not in ('revolute', 'prismatic'):
            raise ValueError('ROBOT_MODEL_JOINT_UNSUPPORTED:' + name)
        limit = joint.find('limit')
        try:
            lo, hi, speed = (float(limit.get(k)) for k in ('lower', 'upper', 'velocity'))
            axis = joint.find('axis')
            vector = [float(x) for x in axis.get('xyz', '1 0 0').split()] if axis is not None else [1., 0., 0.]
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError('ROBOT_MODEL_LIMIT:' + name) from exc
        if (not all(math.isfinite(v) for v in (lo, hi, speed, *vector)) or
                lo >= hi or speed <= 0 or len(vector) != 3 or sum(v*v for v in vector) < 1e-12):
            raise ValueError('ROBOT_MODEL_LIMIT:' + name)
        bounds[name] = (lo, hi, speed)
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError('ROBOT_MODEL_ROOT')
    for link in links:
        seen, current = set(), link
        while current in parents:
            if current in seen:
                raise ValueError('ROBOT_MODEL_CYCLE')
            seen.add(current)
            current = parents[current]
    return links, bounds, next(iter(roots))


@dataclass(frozen=True)
class RobotSetup:
    robot_id: str
    path: Path
    profile_path: Path
    urdf: Path
    display_urdf: Path
    base_frame: str
    single_arm: dict
    tool_config: Path
    profile: Profile
    active_joints: tuple
    context_joints: tuple
    digest: str

    @classmethod
    def load(cls, path, packages=None, overrides=None):
        path = Path(path).resolve()
        raw = json.loads(path.read_text())
        if (set(raw) != {'schema_version', 'robot_id', 'model', 'profile', 'teleop', 'position_mapping'} or
                type(raw['schema_version']) is not int or raw['schema_version'] != 1 or
                not isinstance(raw['robot_id'], str) or not raw['robot_id']):
            raise ValueError('ROBOT_CONFIG_SCHEMA')
        missing = [key for key in ('model', 'profile', 'teleop') if raw[key] is None]
        if missing:
            raise ValueError('ROBOT_CONFIG_INCOMPLETE:' + raw['robot_id'] + ':' + ','.join(missing))
        if overrides:
            if set(overrides) != {'profile', 'tool'}:
                raise ValueError('ROBOT_CONFIG_OVERRIDES')
            raw['profile'] = overrides['profile']
            raw['teleop']['tool'] = overrides['tool']
        model, teleop = raw['model'], raw['teleop']
        if (set(model) != {'urdf', 'display_urdf', 'base_frame'} or
                set(teleop) != {'single_arm', 'tool'} or
                not teleop['single_arm'] or set(teleop['single_arm']) - {'left', 'right'}):
            raise ValueError('ROBOT_CONFIG_SCHEMA')
        profile_path = resolve_file(raw['profile'], path, packages)
        profile = Profile.load(profile_path)
        urdf = resolve_file(model['urdf'], path, packages)
        display = resolve_file(model['display_urdf'], path, packages)
        links, bounds, base = model_layout(urdf)
        if model['base_frame'] != base:
            raise ValueError('ROBOT_BASE_FRAME_MISMATCH')
        names = {n for group in profile.groups.values() for n in group}
        if names != set(bounds):
            raise ValueError('ROBOT_MODEL_PROFILE_LAYOUT:missing_feedback=' +
                             ','.join(sorted(set(bounds)-names)) + ';unknown=' +
                             ','.join(sorted(names-set(bounds))))
        for name, (lo, hi, speed) in bounds.items():
            low, high = profile.limits[name]
            if low < lo-1e-6 or high > hi+1e-6 or profile.speed(name) > speed+1e-6:
                raise ValueError('ROBOT_LIMIT_EXCEEDS_MODEL:' + name)
        display_links, display_bounds, display_base = model_layout(display)
        if base != display_base or not names <= set(display_bounds):
            raise ValueError('ROBOT_DISPLAY_LAYOUT')
        single = {side: resolve_file(value, path, packages)
                  for side, value in teleop['single_arm'].items()}
        for side, config_path in single.items():
            config = load_config(config_path, profile, urdf.parent)
            kin = config['kinematics']
            if (config['side'] != side or Path(kin['urdf']).resolve() != urdf or
                    kin['ee_link'] not in links or kin.get('measured_context') is not True or
                    set(kin['locked_joints']) != names-set(kin['joint_names'])):
                raise ValueError('ROBOT_SINGLE_ARM_MODEL:' + side)
        tool_path = resolve_file(teleop['tool'], path, packages)
        tool = json.loads(tool_path.read_text())
        if ('input_frame' in tool and tool['input_frame'] != base):
            raise ValueError('ROBOT_TOOL_INPUT_FRAME')
        groups = tool['resource_groups']
        if (len(groups) not in (2, 3) or len(groups) != len(set(groups)) or
                any(g not in profile.groups or not profile.groups[g] for g in groups)):
            raise ValueError('ROBOT_TOOL_GROUPS')
        if (Path(urdf.parent / tool['urdf']).resolve() != urdf or
                tool['left_frame'] not in links or tool['right_frame'] not in links):
            raise ValueError('ROBOT_TOOL_MODEL')
        for side, group in zip(('left', 'right'), groups[-2:]):
            if side in single and json.loads(single[side].read_text())['resource_group'] != group:
                raise ValueError('ROBOT_TOOL_SIDE:' + side)
        active = tuple(n for group in groups for n in profile.groups[group])
        context = tuple(sorted(names-set(active)))
        # Include each semantic input once; no dependence on installation prefix.
        files = [path, profile_path, urdf, display, tool_path, *single.values()]
        for config_path in single.values():
            collision = json.loads(config_path.read_text())['kinematics'].get('collision_model')
            if collision:
                files.append(resolve_file(collision, config_path, packages))
        if raw['position_mapping'] is not None:
            from bindu_hardware.position_mapping import PositionMapping
            mapping = resolve_file(raw['position_mapping'], path, packages)
            PositionMapping(json.loads(mapping.read_text()), names)
            files.append(mapping)
        digest = hashlib.sha256(b'\0'.join(p.read_bytes() for p in files)).hexdigest()
        return cls(raw['robot_id'], path, profile_path, urdf, display, base,
                   single, tool_path, profile, active, context, digest)

    def summary(self):
        return {'robot': self.robot_id, 'configuration_sha256': self.digest,
                'model_dof': sum(map(len, self.profile.groups.values())),
                'tool_dof': len(self.active_joints),
                'groups': self.profile.groups, 'tool_joints': self.active_joints,
                'measured_context_joints': self.context_joints,
                'hardware_validated': False}


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Validate a robot configuration without starting ROS or hardware')
    parser.add_argument('config')
    args = parser.parse_args()
    print(json.dumps(RobotSetup.load(args.config).summary(), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
