"""Validate a small deployment recipe. No hardware enable or startup motion."""
import json
import math
from pathlib import Path
from bindu_contracts.runtime_mode import resolve_mode


COMMON = {'namespace', 'profile', 'bimanual_config', 'side', 'tool_input_source',
          'vr_enabled', 'pico_enabled', 'pico_simulation_demo', 'pico_config', 'host', 'port', 'cert_file', 'key_file', 'recording_mode',
          'single_arm_enabled','recorder_enabled','command_enabled','vendor_namespace',
          'arm_receiver_node','servo_receiver_node','mid_receiver_node',
          'output', 'device_backend', 'description_enabled', 'tool_display_enabled', 'runtime_mode'}
EXECUTION = {'runtime_mode', 'namespace', 'profile', 'device_backend', 'run_id', 'output',
             'recording_mode', 'command_enabled', 'vendor_namespace', 'arm_receiver_node',
             'servo_receiver_node', 'mid_receiver_node'}
COMMON.add('gripper_config')
COMMON.update(('position_protocol','execution_mode'))
COMMON.update(('position_contract','physics_config'))
EXECUTION.add('gripper_config')
EXECUTION.update(('position_protocol','execution_mode'))
TARGET_STREAM_LIMIT_DEFAULTS = {'trajectory_max_velocity': .25, 'trajectory_max_acceleration': .5}
COMMON.update(TARGET_STREAM_LIMIT_DEFAULTS)
EXECUTION.update(TARGET_STREAM_LIMIT_DEFAULTS)
PATHS = {'profile', 'bimanual_config', 'robot_config', 'cert_file', 'key_file', 'output', 'pico_config', 'gripper_config', 'physics_config'}


def target_stream_limits(values):
    """Startup ceilings, paired with servo parameters; profile limits still apply."""
    result = {}
    for key, default in TARGET_STREAM_LIMIT_DEFAULTS.items():
        raw = values.get(key, default)
        try:
            value = float(raw) if type(raw) in (int, float, str) else float('nan')
        except (ValueError, OverflowError):
            value = float('nan')
        if not math.isfinite(value) or value <= 0:
            raise ValueError('SNOWBOT_TARGET_DYNAMIC_LIMIT:'+key)
        result[key] = value
    return result


def load_deployment(path, share):
    path, share = Path(path).resolve(), Path(share).resolve()
    data = json.loads(path.read_text())
    version = data.get('schema_version')
    if (set(data) != {'schema_version', 'robot_launch', 'arguments'} or
            type(version) is not int or version not in (1, 2)):
        raise ValueError('TELEOP_DEPLOYMENT_SCHEMA')
    launch = data['robot_launch']
    # Add robot assemblies when they expose the same bounded teleop contract.
    execution_only = version == 2 and launch == 'snowbot_adapter.launch.py'
    if not execution_only and launch != ('snowbot_sim.launch.py' if version == 1 else 'robot_teleop.launch.py'):
        raise ValueError('TELEOP_DEPLOYMENT_ROBOT_UNSUPPORTED')
    args = data['arguments']
    allowed = EXECUTION if execution_only else (COMMON if version == 1 else COMMON-{'profile', 'bimanual_config'} | {'robot_config'})
    if not isinstance(args, dict) or set(args)-allowed:
        raise ValueError('TELEOP_DEPLOYMENT_ARGUMENT')
    if args.get('position_protocol', 'legacy_guard') not in ('legacy_guard', 'session_v1', 'session_v3'):
        raise ValueError('SNOWBOT_POSITION_PROTOCOL')
    configured_limits = {key: args[key] for key in TARGET_STREAM_LIMIT_DEFAULTS if key in args}
    if configured_limits:
        if args.get('runtime_mode') != 'real' or args.get('position_protocol') != 'session_v3':
            raise ValueError('SNOWBOT_TARGET_LIMITS_REQUIRE_TARGET_STREAM')
        configured_limits = target_stream_limits(configured_limits)
    execution_mode = args.get('execution_mode', 'position')
    if execution_mode not in ('position', 'impedance'):
        raise ValueError('SNOWBOT_EXECUTION_MODE')
    if execution_mode == 'impedance' and (args.get('runtime_mode') != 'real' or args.get('position_protocol') != 'session_v1'):
        raise ValueError('IMPEDANCE_REQUIRES_REAL_SESSION')
    required = {'runtime_mode'} if execution_only else ({'profile', 'bimanual_config'} if version == 1 else {'robot_config'})
    if not required <= set(args):
        raise ValueError('TELEOP_DEPLOYMENT_MODEL_REQUIRED')
    if 'runtime_mode' in args and args['runtime_mode'] not in ('simulation', 'real'):
        raise ValueError('RUNTIME_MODE_INVALID')
    selected_mode = resolve_mode(args.get('runtime_mode', 'simulation'))
    position_contract=args.get('position_contract',False)
    if type(position_contract) is not bool:
        raise ValueError('TELEOP_DEPLOYMENT_BOOLEAN:position_contract')
    if args.get('gripper_config') and selected_mode != 'real' and not position_contract:
        raise ValueError('GRIPPER_REQUIRES_REAL_MODE')
    backend = args.get('device_backend', '') if execution_only else args.get('device_backend', 'kinematic')
    allowed_backends = ('', 'snowbot_vendor') if selected_mode == 'real' else ('', 'kinematic', 'external_simulation')
    if backend not in allowed_backends:
        raise ValueError('RUNTIME_MODE_BACKEND_CONFLICT')
    if selected_mode == 'real' and not execution_only and (
            version!=2 or args.get('single_arm_enabled',True)):
        raise ValueError('REAL_REQUIRES_SNOWBOT_BIMANUAL_POSITION')
    if args.get('recorder_enabled',True) is False and args.get('recording_mode','compact')!='off':
        raise ValueError('RECORDER_DISABLED_REQUIRES_OFF')
    if args.get('tool_input_source', 'tool_pose') not in ('vr', 'tool_pose'):
        raise ValueError('TELEOP_DEPLOYMENT_INPUT')
    if position_contract and (version!=2 or execution_only or backend not in ('kinematic','snowbot_vendor','external_simulation')):
        raise ValueError('POSITION_CONTRACT_BACKEND')
    if args.get('physics_config') and (selected_mode!='simulation' or backend!='external_simulation' or not position_contract):
        raise ValueError('PHYSICS_REQUIRES_SIMULATION_POSITION_BACKEND')
    if args.get('pico_enabled', False) and (args.get('tool_input_source') != 'vr' or
            (backend != 'kinematic' and not position_contract)):
        raise ValueError('PICO_REQUIRES_KINEMATIC_VR')
    if 'pico_simulation_demo' in args and type(args['pico_simulation_demo']) is not bool:
        raise ValueError('TELEOP_DEPLOYMENT_BOOLEAN:pico_simulation_demo')
    if args.get('pico_simulation_demo',False) and (position_contract or not args.get('pico_enabled',False) or
            selected_mode!='simulation' or backend!='kinematic'):
        raise ValueError('PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION')
    if args.get('recording_mode', 'compact') not in ('off', 'compact', 'normal'):
        raise ValueError('TELEOP_DEPLOYMENT_RECORDING')
    result = {'runtime_mode': selected_mode} if execution_only else {
        'runtime_mode': selected_mode, 'tool_input_source': 'tool_pose', 'vr_enabled': 'false',
        'initialize_tool_pose': 'false'}
    if version == 2 and not execution_only:
        del result['initialize_tool_pose']
    for key, value in args.items():
        if key in configured_limits:
            result[key] = str(configured_limits[key])
            continue
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


def apply_execution_tracking(config, mode='position', *, real, position_contract,
                             trajectory_reference=False):
    """Pair accumulated planning with nominal error; measured guards stay active."""
    if mode not in ('position', 'impedance'):
        raise ValueError('SNOWBOT_EXECUTION_MODE')
    if mode == 'position' and not trajectory_reference:
        return config
    if not real or not position_contract or config.get('reference_kinematics') is not True:
        raise ValueError(('TRAJECTORY' if mode=='position' else 'IMPEDANCE')+
                         '_REQUIRES_REAL_NOMINAL_KINEMATICS')
    return {**config, 'tracking_feedback': 'reference'}
