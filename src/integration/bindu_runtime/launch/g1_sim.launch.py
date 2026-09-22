"""G1 simulation: measured TF, site navigation, optional single-arm VR and Pi."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def assemble(context):
    share = Path(get_package_share_directory('bindu_runtime'))
    model = Path(get_package_share_directory('bindu_description'))/'urdf/g1_provisional.urdf'
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise ValueError('G1_SIDE_MUST_BE_LEFT_OR_RIGHT')
    common = {'profile': LaunchConfiguration('profile'), 'run_id': LaunchConfiguration('run_id')}
    nodes = [Node(package='bindu_runtime', executable=name, namespace=LaunchConfiguration('namespace'),
                  parameters=[{**common, **({'output': LaunchConfiguration('output')} if name == 'recorder' else {}),
                               **({'device_backend': LaunchConfiguration('device_backend')} if name == 'execution' else {})}],
                  remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen')
             for name in ('execution', 'recorder', 'description_state')]
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      namespace=LaunchConfiguration('namespace'), output='screen',
                      parameters=[{'robot_description': model.read_text(), 'publish_frequency': 30.}],
                      remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    ns = LaunchConfiguration('namespace')
    teleop = {**common, 'teleop_config': str(share/f'config/teleop_g1_{side}.json'),
              'model_root': str(model.parent)}
    enabled = (LaunchConfiguration('teleop_enabled').perform(context).lower() == 'true' or
               LaunchConfiguration('vr_enabled').perform(context).lower() == 'true')
    if enabled:
        for name in ('teleop', 'teleop_display'):
            nodes.append(Node(package='bindu_runtime', executable=name, namespace=ns,
                              output='screen', parameters=[teleop]))
    from launch_ros.parameter_descriptions import ParameterValue
    vr = {k:ParameterValue(LaunchConfiguration(k), value_type=str) for k in ('host','side','cert_file','key_file')}
    vr['port'] = ParameterValue(LaunchConfiguration('port'), value_type=int)
    nodes.append(Node(package='bindu_runtime', executable='vr_input', namespace=ns, output='screen',
                      condition=IfCondition(LaunchConfiguration('vr_enabled')), parameters=[{**common, **vr}]))
    nodes.append(Node(package='bindu_runtime', executable='pi_client', namespace=ns, output='screen',
                      condition=IfCondition(LaunchConfiguration('pi_enabled')),
                      parameters=[{**common, 'pi_config': LaunchConfiguration('pi_config')}]))
    nodes.append(Node(package='bindu_runtime', executable='task', namespace=ns, output='screen',
                      condition=IfCondition(LaunchConfiguration('navigation_enabled')),
                      parameters=[{**common, 'navigation_provider': 'bindu_runtime.navigation:Nav2Navigation',
                                   'navigation_config': LaunchConfiguration('navigation_config'),
                                   'navigation_backend': LaunchConfiguration('navigation_backend')}]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'namespace':'bindu_g1_sim', 'run_id':uuid.uuid4().hex, 'output':'/tmp/bindu-runs',
                'profile':str(share/'config/g1_provisional_sim.json'), 'device_backend':'kinematic',
                'teleop_enabled':'false', 'vr_enabled':'false', 'side':'left', 'host':'127.0.0.1',
                'port':'8012', 'cert_file':'', 'key_file':'', 'pi_enabled':'false',
                'pi_config':str(share/'config/pi_g1.json'), 'navigation_enabled':'true',
                'navigation_config':str(share/'config/navigation_sim.json'),
                'navigation_backend':'navigation/backend'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+
                             [OpaqueFunction(function=assemble)])
