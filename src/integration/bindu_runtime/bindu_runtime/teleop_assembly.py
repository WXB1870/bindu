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


def assemble(context):
    value = lambda name, default='': LaunchConfiguration(name, default=default).perform(context)
    mode = resolve_mode(value('runtime_mode', 'simulation'))
    if mode != 'simulation':
        raise ValueError('ROBOT_TELEOP_REAL_BACKEND_NOT_IMPLEMENTED')
    legacy = not value('robot_config')
    selection = value('robot_config') or str(Path(get_package_share_directory('bindu_runtime'))/'config/robots/snowbot/robot_snowbot.json')
    overrides = {'profile': value('profile'), 'tool': value('bimanual_config')} if legacy else None
    robot = RobotSetup.load(selection, overrides=overrides)
    side, backend = value('side'), value('device_backend')
    if side not in robot.single_arm:
        raise ValueError('ROBOT_SINGLE_ARM_UNAVAILABLE:' + side)
    # Hardware selection is explicit. Neither an unfamiliar robot nor an
    # unsupported backend can silently become a simulated device.
    if backend not in ('kinematic', 'external_simulation'):
        raise ValueError('ROBOT_HARDWARE_BACKEND_NOT_IMPLEMENTED:' + backend)
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
    if pico and (robot.robot_id != 'snowbot' or backend != 'kinematic' or value('tool_input_source') != 'vr'):
        raise ValueError('PICO_REQUIRES_SNOWBOT_KINEMATIC_VR')
    pico_config = value('pico_config') or str(Path(get_package_share_directory('bindu_runtime'))/'config/robots/snowbot/pico_snowbot_sim.json')
    if pico:
        tool_settings=json.loads(robot.tool_config.read_text())
        if not tool_settings.get('right_axial_grasp') or tool_settings.get('collision_scope')!='configured_tool_geometry' or not tool_settings.get('demo',{}).get('initial_positions_rad'):
            raise ValueError('PICO_REQUIRES_AXIAL_SHOVEL_CONFIG')
        settings=json.loads(Path(pico_config).read_text())
        if settings.get('simulation_only') is not True or settings.get('mapping')!='pico_snowbot_v1':
            raise ValueError('PICO_CONFIG_UNAVAILABLE')
    ns = LaunchConfiguration('namespace')
    common = {'profile': str(robot.profile_path), 'run_id': LaunchConfiguration('run_id'), 'runtime_mode': mode,
              **({} if legacy else {'robot_config': str(robot.path), 'robot_config_digest': robot.digest})}
    model = {'model_root': str(robot.urdf.parent)}
    nodes = [LogInfo(msg=f'Robot {robot.robot_id}: {len(robot.active_joints)} tool DOF, '
                     f'{len(robot.context_joints)} measured context joints; config={robot.digest}')]
    execution = {**common, 'device_backend': backend}
    if initial:
        pose = json.loads(robot.tool_config.read_text())['demo']['initial_positions_rad']
        execution['kinematic_initial_positions'] = ParameterValue(json.dumps(pose), value_type=str)
    nodes.append(Node(package='bindu_runtime', executable='execution', namespace=ns,
                      output='screen', parameters=[execution]))
    # Even with file writing disabled, sessions require RecorderHealth.
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
                          'supported_sides': [side], 'require_single_worker': True}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop', namespace=ns,
                      output='screen', parameters=[{**common, **model,
                          'teleop_config': str(robot.single_arm[side]), 'require_mode_selection': True}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_tool', namespace=ns,
                      output='screen', parameters=[{**common, **model, **tool}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_display', namespace=ns,
                      condition=IfCondition(LaunchConfiguration('tool_display_enabled')),
                      output='screen', parameters=[{**common, **model, 'tool_config': str(robot.tool_config)}]))
    vr = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
          for key in ('host', 'cert_file', 'key_file')}
    if pico:
        nodes.append(Node(package='bindu_runtime',executable='pico_control',namespace=ns,output='screen',
            parameters=[{**common,**model,'tool_config':str(robot.tool_config),
                         'pico_config':pico_config,'device_backend':backend}]))
    nodes.append(Node(package='bindu_runtime', executable='vr_input', namespace=ns,
                      condition=IfCondition(LaunchConfiguration('vr_enabled')), output='screen',
                      parameters=[{**common, **vr, 'side': 'both', 'mapping': 'pico_snowbot_v1' if pico else 'v34',
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}],
                      remappings=[('teleop/vr/input','teleop/vr/raw')] if pico else []))
    return nodes
