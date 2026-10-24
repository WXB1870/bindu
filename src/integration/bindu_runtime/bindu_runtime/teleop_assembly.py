"""Robot-selected teleoperation. Validation finishes before any node starts."""
from pathlib import Path
import json
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from bindu_runtime.robot_setup import RobotSetup
from bindu_contracts.runtime_mode import resolve_mode
from bindu_runtime.teleop_deployment import TARGET_STREAM_LIMIT_DEFAULTS, target_stream_limits


def assemble(context):
    value = lambda name, default='': LaunchConfiguration(name, default=default).perform(context)
    def enabled(name, default='true'):
        raw=value(name,default)
        if raw not in ('true','false'):raise ValueError('TELEOP_BOOLEAN:'+name)
        return raw=='true'
    single_enabled=enabled('single_arm_enabled')
    recorder_enabled=enabled('recorder_enabled')
    if not recorder_enabled and value('recording_mode')!='off':
        raise ValueError('RECORDER_DISABLED_REQUIRES_OFF')
    mode = resolve_mode(value('runtime_mode', 'simulation'))
    position_contract=enabled('position_contract','false')
    execution_mode=value('execution_mode','position')
    if execution_mode not in ('position','impedance'):
        raise ValueError('SNOWBOT_EXECUTION_MODE')
    if execution_mode=='impedance' and (mode!='real' or value('position_protocol') != 'session_v1' or not position_contract):
        raise ValueError('IMPEDANCE_REQUIRES_REAL_SESSION')
    if position_contract and value('device_backend') not in ('kinematic','snowbot_vendor','external_simulation'):
        raise ValueError('POSITION_CONTRACT_BACKEND')
    if value('gripper_config') and mode != 'real' and not position_contract:
        raise ValueError('GRIPPER_REQUIRES_REAL_MODE')
    legacy = not value('robot_config')
    selection = value('robot_config') or str(Path(get_package_share_directory('bindu_runtime'))/'config/robots/snowbot/robot_snowbot.json')
    overrides = {'profile': value('profile'), 'tool': value('bimanual_config')} if legacy else None
    robot = RobotSetup.load(selection, overrides=overrides)
    if position_contract and (robot.robot_id!='snowbot' or single_enabled):
        raise ValueError('POSITION_CONTRACT_REQUIRES_SNOWBOT_BIMANUAL')
    side, backend = value('side'), value('device_backend')
    if single_enabled and side not in robot.single_arm:
        raise ValueError('ROBOT_SINGLE_ARM_UNAVAILABLE:' + side)
    # Hardware selection is explicit. Neither an unfamiliar robot nor an
    # unsupported backend can silently become a simulated device.
    if backend not in (('snowbot_vendor',) if mode=='real' else ('kinematic','external_simulation')):
        raise ValueError('ROBOT_HARDWARE_BACKEND_NOT_IMPLEMENTED:' + backend)
    if mode=='real' and (robot.robot_id!='snowbot' or single_enabled):
        raise ValueError('REAL_REQUIRES_SNOWBOT_BIMANUAL_POSITION')
    tool_settings=json.loads(robot.tool_config.read_text())
    if tool_settings.get('simulation_only') is not (mode=='simulation' and not position_contract):
        raise ValueError('BIMANUAL_CONFIG_RUNTIME_MODE_MISMATCH')
    if value('tool_input_source') not in ('tool_pose', 'vr'):
        raise ValueError('ROBOT_TOOL_INPUT')
    if (value('tool_input_source') == 'tool_pose' and
            json.loads(robot.tool_config.read_text()).get('input_frame') != robot.base_frame):
        raise ValueError('ROBOT_TOOL_INPUT_FRAME')
    if value('recording_mode') not in ('off', 'compact', 'normal'):
        raise ValueError('ROBOT_RECORDING_MODE')
    initial = value('initialize_tool_pose', 'false').lower() == 'true'
    if initial and (not legacy or backend != 'kinematic'):
        raise ValueError('INITIAL_POSE_ONLY_FOR_KINEMATIC_FIXTURE')
    pico = value('pico_enabled','false').lower() == 'true'
    pico_demo=enabled('pico_simulation_demo','false')
    if pico_demo and (position_contract or not pico or mode!='simulation' or backend!='kinematic'):
        raise ValueError('PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION')
    if pico and (robot.robot_id != 'snowbot' or (backend != 'kinematic' and not position_contract) or value('tool_input_source') != 'vr'):
        raise ValueError('PICO_REQUIRES_SNOWBOT_KINEMATIC_VR')
    pico_config = value('pico_config') or str(Path(get_package_share_directory('bindu_runtime'))/'config/robots/snowbot/pico_snowbot_sim.json')
    base_config = value('base_config')
    if base_config:
        if not pico or not position_contract or not enabled('vr_enabled','false'):
            raise ValueError('BASE_REQUIRES_PICO_POSITION_VR')
        from bindu_teleoperation.base import validate_base_config
        validate_base_config(json.loads(Path(base_config).read_text()))
    if pico:
        tool_settings=json.loads(robot.tool_config.read_text())
        if not tool_settings.get('right_axial_grasp') or tool_settings.get('collision_scope')!='configured_tool_geometry' or (not position_contract and not tool_settings.get('demo',{}).get('initial_positions_rad')):
            raise ValueError('PICO_REQUIRES_AXIAL_SHOVEL_CONFIG')
        settings=json.loads(Path(pico_config).read_text())
        if (settings.get('simulation_only') is not (mode=='simulation') or settings.get('mapping')!='pico_snowbot_v1' or
                settings.get('schema_version') != (2 if position_contract else 1)):
            raise ValueError('PICO_CONFIG_UNAVAILABLE')
        if position_contract and not value('gripper_config'):
            raise ValueError('PICO_GRIPPER_CONFIG_REQUIRED')
        if position_contract:
            from bindu_hardware.snowbot_gripper import validate_config
            grippers=validate_config(json.loads(Path(value('gripper_config')).read_text()),robot.profile)
            if grippers.get('simulation_only',False) is not (mode=='simulation'):
                raise ValueError('PICO_GRIPPER_CONFIG_MODE_MISMATCH')
        if mode=='real' and settings.get('device_initial_positions_rad'):
            raise ValueError('REAL_REQUIRES_MEASURED_INITIAL_STATE')
    ns = LaunchConfiguration('namespace')
    common = {'profile': str(robot.profile_path), 'run_id': LaunchConfiguration('run_id'), 'runtime_mode': mode,
              **({} if legacy else {'robot_config': str(robot.path), 'robot_config_digest': robot.digest})}
    model = {'model_root': str(robot.urdf.parent)}
    nodes = [LogInfo(msg=f'Robot {robot.robot_id}: {len(robot.active_joints)} tool DOF, '
                     f'{len(robot.context_joints)} measured context joints; config={robot.digest}')]
    execution = {**common, 'device_backend': backend}
    if pico and position_contract and backend=='kinematic' and settings.get('device_initial_positions_rad'):
        execution['kinematic_initial_positions'] = ParameterValue(
            json.dumps(settings['device_initial_positions_rad']), value_type=str)
    if initial:
        pose = json.loads(robot.tool_config.read_text())['demo']['initial_positions_rad']
        execution['kinematic_initial_positions'] = ParameterValue(json.dumps(pose), value_type=str)
    if mode=='real' or position_contract:
        execution.update(command_enabled=enabled('command_enabled','false'),
            **{key:value(key,default) for key,default in (
                ('execution_mode','position'),('gripper_config',''),('vendor_namespace','/snowbot'),('position_protocol','legacy_guard'),('arm_receiver_node',''),
                ('servo_receiver_node',''),('mid_receiver_node',''))})
    if mode=='real' and value('position_protocol')=='session_v3':
        execution.update(target_stream_limits({key:value(key,str(default))
            for key,default in TARGET_STREAM_LIMIT_DEFAULTS.items()}))
    nodes.append(Node(package='bindu_runtime', executable='snowbot_adapter' if mode=='real' else 'snowbot_position_sim' if position_contract else 'execution', namespace=ns,
                      output='screen', parameters=[execution]))
    if recorder_enabled:
        nodes.append(Node(package='bindu_runtime', executable='recorder', namespace=ns,
                          output='screen', parameters=[{**common,
                              'output': LaunchConfiguration('output'),
                              'recording_mode': ParameterValue(LaunchConfiguration('recording_mode'), value_type=str)}]))
    if value('description_enabled').lower() == 'true':
        nodes.append(Node(package='bindu_runtime', executable='description_state', namespace=ns,
                          output='screen', parameters=[{**common, 'base_frame': robot.base_frame}],
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                          namespace=ns, output='screen', parameters=[{
                              'robot_description': robot.display_urdf.read_text(), 'publish_frequency': 30.}],
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    tool = {'tool_config': str(robot.tool_config), 'input_source': value('tool_input_source')}
    nodes.append(Node(package='bindu_runtime', executable='teleop_mode', namespace=ns,
                      output='screen', parameters=[{**common, **tool, 'initial_side': side,
                          'supported_sides': [side],
                          'single_arm_enabled':single_enabled,'require_single_worker':single_enabled,
                          'independent_arms_enabled':position_contract,
                          'require_recording':recorder_enabled}]))
    if single_enabled:
        nodes.append(Node(package='bindu_runtime', executable='teleop', namespace=ns,
                          output='screen', parameters=[{**common, **model,
                              'teleop_config': str(robot.single_arm[side]), 'require_mode_selection': True,'require_recording':recorder_enabled}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_tool', namespace=ns,
                      output='screen', parameters=[{**common, **model, **tool,'require_recording':recorder_enabled,
                          'simulation_demo':pico_demo,
                          'position_contract':position_contract,
                          'execution_mode':execution_mode,
                          'position_protocol':value('position_protocol','legacy_guard'),
                          'gripper_config':value('gripper_config') if position_contract else '',
                          'tool_ground_policy':'report_only' if pico_demo else 'configured'}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_display', namespace=ns,
                      condition=IfCondition(LaunchConfiguration('tool_display_enabled')),
                      output='screen', parameters=[{**common, **model, 'tool_config': str(robot.tool_config)}]))
    vr = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
          for key in ('host', 'cert_file', 'key_file')}
    if pico:
        nodes.append(Node(package='bindu_runtime',executable='pico_control',namespace=ns,output='screen',
            parameters=[{**common,**model,'tool_config':str(robot.tool_config),
                         'pico_config':pico_config,'device_backend':backend,'recording_enabled':recorder_enabled,
                         'gripper_config':value('gripper_config'),
                         'base_config':base_config, 'command_enabled':enabled('command_enabled','false'),
                         'simulation_demo':pico_demo}]))
    nodes.append(Node(package='bindu_runtime', executable='vr_input', namespace=ns,
                      condition=IfCondition(LaunchConfiguration('vr_enabled')), output='screen',
                      parameters=[{**common, **vr, 'side': 'both', 'mapping': 'pico_snowbot_v1' if pico or mode=='real' else 'v34',
                          'input_timing_enabled':ParameterValue(LaunchConfiguration('input_timing_enabled',default='false'),value_type=bool),
                          'simulation_demo':pico_demo,
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}],
                      remappings=[('teleop/vr/input','teleop/vr/raw')] if pico else []))
    return nodes
