"""Snowbot CAD fixture: manual single-arm / dual-arm tool simulation."""
from pathlib import Path
import uuid
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def assemble(context):
    share = Path(get_package_share_directory('bindu_runtime'))
    description = Path(get_package_share_directory('bindu_description')) / 'urdf'
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise ValueError('SNOWBOT_SIDE_MUST_BE_LEFT_OR_RIGHT')
    namespace = LaunchConfiguration('namespace')
    common = {'profile': LaunchConfiguration('profile'), 'run_id': LaunchConfiguration('run_id')}
    nodes = []
    description_enabled = LaunchConfiguration('description_enabled').perform(context).lower() == 'true'
    for name in ('execution', 'recorder', 'description_state'):
        if name == 'description_state' and not description_enabled:
            continue
        parameters = dict(common)
        if name == 'execution':
            parameters['device_backend'] = LaunchConfiguration('device_backend')
            if LaunchConfiguration('initialize_tool_pose').perform(context).lower() == 'true':
                fixture = json.loads(Path(LaunchConfiguration('bimanual_config').perform(context)).read_text())
                parameters['kinematic_initial_positions'] = ParameterValue(
                    json.dumps(fixture['demo']['initial_positions_rad']), value_type=str)
        if name == 'description_state':
            parameters['base_frame'] = 'car_base_link'
        if name == 'recorder':
            parameters.update(output=LaunchConfiguration('output'),
                              recording_mode=ParameterValue(
                                  LaunchConfiguration('recording_mode'), value_type=str))
        nodes.append(Node(package='bindu_runtime', executable=name, namespace=namespace,
                          parameters=[parameters], output='screen',
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      namespace=namespace, output='screen',
                      condition=IfCondition(LaunchConfiguration('description_enabled')),
                      parameters=[{'robot_description': (description / 'snowbot_display.urdf').read_text(),
                                   'publish_frequency': 30.}],
                      remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_mode', namespace=namespace,
                      output='screen', parameters=[{**common, 'initial_side': side,
                                                    'supported_sides': [side],
                                                    'require_single_worker': True,
                                                    'input_source': LaunchConfiguration('tool_input_source'),
                                                    'tool_config': LaunchConfiguration('bimanual_config')}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop', namespace=namespace,
                      output='screen', parameters=[{**common,
                          'teleop_config': str(share / f'config/teleop_snowbot_{side}.json'),
                          'model_root': str(description), 'require_mode_selection': True}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_tool', namespace=namespace,
                      output='screen', parameters=[{**common,
                          'input_source': LaunchConfiguration('tool_input_source'),
                          'tool_config': LaunchConfiguration('bimanual_config'),
                          'model_root': str(description)}]))
    nodes.append(Node(package='bindu_runtime', executable='vr_input', namespace=namespace,
                      condition=IfCondition(LaunchConfiguration('vr_enabled')), output='screen',
                      parameters=[{**common, 'side': 'both',
                          'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          'cert_file': ParameterValue(LaunchConfiguration('cert_file'), value_type=str),
                          'key_file': ParameterValue(LaunchConfiguration('key_file'), value_type=str)}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_display', namespace=namespace,
                      condition=IfCondition(LaunchConfiguration('tool_display_enabled')),
                      parameters=[{**common, 'tool_config': LaunchConfiguration('bimanual_config'),
                                   'model_root': str(description)}], output='screen'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'namespace': 'bindu_snowbot_sim', 'run_id': uuid.uuid4().hex,
                'output': '/tmp/bindu-snowbot-runs', 'recording_mode': 'compact',
                'profile': str(share / 'config/snowbot_sim.json'),
                'bimanual_config': str(share / 'config/bimanual_snowbot_sim.json'),
                'device_backend': 'kinematic', 'side': 'left', 'initialize_tool_pose': 'false',
                'tool_input_source': 'vr',
                'description_enabled': 'true', 'tool_display_enabled': 'false',
                'vr_enabled': 'false', 'host': '127.0.0.1', 'port': '8012',
                'cert_file': '', 'key_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, v in defaults.items()] + [OpaqueFunction(function=assemble)])
