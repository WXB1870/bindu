"""Validate a small deployment recipe. No hardware enable or startup motion."""
import json
from pathlib import Path


COMMON = {'namespace', 'profile', 'bimanual_config', 'side', 'tool_input_source',
          'vr_enabled', 'host', 'port', 'cert_file', 'key_file', 'recording_mode',
          'output', 'device_backend', 'description_enabled', 'tool_display_enabled'}
PATHS = {'profile', 'bimanual_config', 'cert_file', 'key_file', 'output'}


def load_deployment(path, share):
    path, share = Path(path).resolve(), Path(share).resolve()
    data = json.loads(path.read_text())
    if set(data) != {'schema_version', 'robot_launch', 'arguments'} or data['schema_version'] != 1:
        raise ValueError('TELEOP_DEPLOYMENT_SCHEMA')
    launch = data['robot_launch']
    # Add robot assemblies when they expose the same bounded teleop contract.
    if launch != 'snowbot_sim.launch.py':
        raise ValueError('TELEOP_DEPLOYMENT_ROBOT_UNSUPPORTED')
    args = data['arguments']
    if not isinstance(args, dict) or set(args)-COMMON:
        raise ValueError('TELEOP_DEPLOYMENT_ARGUMENT')
    if not {'profile', 'bimanual_config'} <= set(args):
        raise ValueError('TELEOP_DEPLOYMENT_MODEL_REQUIRED')
    if args.get('device_backend', 'kinematic') not in ('kinematic', 'external_simulation'):
        raise ValueError('TELEOP_HARDWARE_ADAPTER_NOT_IMPLEMENTED')
    if args.get('tool_input_source', 'tool_pose') not in ('vr', 'tool_pose'):
        raise ValueError('TELEOP_DEPLOYMENT_INPUT')
    if args.get('recording_mode', 'compact') not in ('off', 'compact', 'normal'):
        raise ValueError('TELEOP_DEPLOYMENT_RECORDING')
    result = {'tool_input_source': 'tool_pose', 'vr_enabled': 'false',
              'initialize_tool_pose': 'false'}
    for key, value in args.items():
        if not isinstance(value, (str, bool, int)):
            raise ValueError('TELEOP_DEPLOYMENT_VALUE:'+key)
        if key.endswith('_enabled') and type(value) is not bool:
            raise ValueError('TELEOP_DEPLOYMENT_BOOLEAN:'+key)
        if key in PATHS and value:
            if not isinstance(value, str):
                raise ValueError('TELEOP_DEPLOYMENT_PATH:'+key)
            # @runtime/ resolves against the installed package, otherwise config-relative.
            resolved = share/value[9:] if value.startswith('@runtime/') else path.parent/value
            value = str(resolved.resolve())
            if key != 'output' and not Path(value).is_file():
                raise ValueError('TELEOP_DEPLOYMENT_MISSING:'+key)
        result[key] = str(value).lower() if type(value) is bool else str(value)
    return launch, result
