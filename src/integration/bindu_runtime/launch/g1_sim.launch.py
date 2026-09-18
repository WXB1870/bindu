"""19-axis joint execution and measured TF. No grasp task or hardware startup."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    model = Path(get_package_share_directory('bindu_description'))/'urdf/g1_provisional.urdf'
    defaults = {'namespace': 'bindu_g1_sim', 'run_id': uuid.uuid4().hex, 'output': '/tmp/bindu-runs'}
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    common = {'profile': str(share/'config/g1_provisional_sim.json'), 'run_id': LaunchConfiguration('run_id')}
    nodes = [Node(package='bindu_runtime', executable=name, namespace=LaunchConfiguration('namespace'),
                  parameters=[{**common, **({'output': LaunchConfiguration('output')} if name == 'recorder' else {})}],
                  remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen')
             for name in ('execution', 'recorder', 'description_state')]
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      namespace=LaunchConfiguration('namespace'), output='screen',
                      parameters=[{'robot_description': model.read_text(), 'publish_frequency': 30.}],
                      remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    return LaunchDescription(args+nodes)
