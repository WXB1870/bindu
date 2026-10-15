"""G1 simulation: measured TF, site navigation, optional single-arm VR and Pi."""
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


def assemble(context):
    mode = resolve_mode(LaunchConfiguration('runtime_mode', default='simulation').perform(context))
    if mode != 'simulation':
        raise ValueError('G1_REAL_BACKEND_NOT_IMPLEMENTED')
    share = Path(get_package_share_directory('bindu_runtime'))
    model = Path(get_package_share_directory('bindu_description'))/'urdf/g1_provisional.urdf'
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise ValueError('G1_SIDE_MUST_BE_LEFT_OR_RIGHT')
    common = {'profile': LaunchConfiguration('profile'), 'run_id': LaunchConfiguration('run_id'), 'runtime_mode': mode}
    nodes = [Node(package='bindu_runtime', executable=name, namespace=LaunchConfiguration('namespace'),
                  parameters=[{**common, **({'output':LaunchConfiguration('output'), 'recording_mode':ParameterValue(LaunchConfiguration('recording_mode'),value_type=str)} if name == 'recorder' else {}),
                               **({'device_backend': LaunchConfiguration('device_backend')} if name == 'execution' else {})}],
                  remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen')
             for name in ('execution', 'recorder', 'description_state')]
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                      namespace=LaunchConfiguration('namespace'), output='screen',
                      parameters=[{'robot_description': model.read_text(), 'publish_frequency': 30.}],
                      remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    ns = LaunchConfiguration('namespace')
    teleop = {**common, 'teleop_config': (LaunchConfiguration('teleop_config').perform(context)
              or str(share/f'config/robots/g1/teleop_g1_{side}.json')),
              'model_root': str(model.parent)}
    bimanual = LaunchConfiguration('bimanual_enabled').perform(context).lower() == 'true'
    enabled = (LaunchConfiguration('teleop_enabled').perform(context).lower() == 'true' or
               LaunchConfiguration('vr_enabled').perform(context).lower() == 'true' or bimanual)
    if enabled:
        for name in ('teleop', 'teleop_display'):
            nodes.append(Node(package='bindu_runtime', executable=name, namespace=ns,
                              output='screen', parameters=[{**teleop,
                                  **({'require_mode_selection': True} if name == 'teleop' and bimanual else {})}]))
    if bimanual:
        nodes.append(Node(package='bindu_runtime', executable='teleop_mode', namespace=ns,
                          output='screen', parameters=[{**common, 'initial_side': side,
                              'supported_sides': [side], 'require_single_worker': True}]))
        nodes.append(Node(package='bindu_runtime', executable='teleop_tool', namespace=ns,
                          output='screen', parameters=[{**common,
                              'tool_config':LaunchConfiguration('bimanual_config'),
                              'model_root':str(model.parent)}]))
    vr = {k:ParameterValue(LaunchConfiguration(k), value_type=str) for k in ('host','cert_file','key_file')}
    vr['side'] = 'both' if bimanual else side
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
    defaults = {'runtime_mode':'simulation', 'namespace':'bindu_g1_sim', 'run_id':uuid.uuid4().hex, 'output':'/tmp/bindu-runs', 'recording_mode':'compact',
                'profile':str(share/'config/robots/g1/g1_provisional_sim.json'), 'device_backend':'kinematic',
                'teleop_enabled':'false', 'teleop_config':'', 'vr_enabled':'false', 'bimanual_enabled':'false',
                'bimanual_config':str(share/'config/robots/g1/bimanual_g1_sim.json'),
                'side':'left', 'host':'127.0.0.1',
                'port':'8012', 'cert_file':'', 'key_file':'', 'pi_enabled':'false',
                'pi_config':str(share/'config/vla/pi_g1.json'), 'navigation_enabled':'true',
                'navigation_config':str(share/'config/navigation/navigation_sim.json'),
                'navigation_backend':'navigation/backend'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+
                             [OpaqueFunction(function=assemble)])
