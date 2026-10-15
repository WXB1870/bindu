"""Single-arm simulation execution; real VR receiver starts only when selected."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from bindu_contracts.runtime_mode import resolve_mode


def validate_mode(context):
    if resolve_mode(LaunchConfiguration('runtime_mode').perform(context)) != 'simulation':
        raise ValueError('TELEOP_REAL_BACKEND_NOT_IMPLEMENTED')
    return []


def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'runtime_mode': 'simulation', 'profile': str(share/'config/robots/huawei_v34/huawei_v34_left_sim.json'),
                'teleop_config': str(share/'config/robots/huawei_v34/teleop_v34.json'),
                'namespace': 'bindu_sim', 'run_id': uuid.uuid4().hex,
                'output': '/tmp/bindu-runs', 'recording_mode': 'compact', 'vr_enabled': 'false',
                'host': '127.0.0.1', 'port': '8012', 'side': 'left',
                'cert_file': '', 'key_file': ''}
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    common = {k: LaunchConfiguration(k) for k in ('profile', 'run_id', 'runtime_mode')}
    nodes = []
    for executable, keys in {'execution': (), 'recorder': ('output','recording_mode'), 'teleop': ('teleop_config',)}.items():
        nodes.append(Node(package='bindu_runtime', executable=executable,
                          namespace=LaunchConfiguration('namespace'), output='screen',
                          parameters=[{**common, **{k:ParameterValue(LaunchConfiguration(k),value_type=str) if k=='recording_mode' else LaunchConfiguration(k) for k in keys}}]))
    vr = {k:ParameterValue(LaunchConfiguration(k), value_type=str)
          for k in ('host', 'side', 'cert_file', 'key_file')}
    vr['port'] = ParameterValue(LaunchConfiguration('port'), value_type=int)
    nodes.append(Node(package='bindu_runtime', executable='vr_input',
                      condition=IfCondition(LaunchConfiguration('vr_enabled')),
                      namespace=LaunchConfiguration('namespace'), output='screen', parameters=[{**common, **vr}]))
    nodes.append(Node(package='bindu_runtime', executable='teleop_display',
                      condition=IfCondition(LaunchConfiguration('vr_enabled')),
                      namespace=LaunchConfiguration('namespace'), output='screen',
                      parameters=[{**common, 'teleop_config': LaunchConfiguration('teleop_config')}]))
    return LaunchDescription(args+[OpaqueFunction(function=validate_mode)]+nodes)
