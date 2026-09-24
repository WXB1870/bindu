"""Composition root: select trusted capability providers without editing task flow."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from bindu_runtime.device_assembly import DEFAULT_DRIVERS


def generate_launch_description():
    defaults = {
        'pi_enabled': 'false',
        'pi_config': str(Path(get_package_share_directory('bindu_runtime'))/'config/pi_loopback.json'),
        'profile': str(Path(get_package_share_directory('bindu_runtime'))/'config/wheel_sim.json'),
        'run_id': uuid.uuid4().hex,
        'namespace': 'bindu_sim',
        'output': '/tmp/bindu-runs',
        'recording_mode': 'compact',
        'planner_provider': 'bindu_planning.simulated:PlannerStrategy',
        'chunk_provider': 'bindu_vla.simulated:ChunkStrategy',
        'navigation_config': str(Path(get_package_share_directory('bindu_runtime'))/'config/navigation_sim.json'),
        'navigation_backend': 'navigation/backend',
        'navigation_provider': 'bindu_navigation.simulated:SimNavigation',
        **DEFAULT_DRIVERS,
        'perception_provider': 'bindu_perception.simulated:SimObjectLocator',
        'perception_package': 'bindu_runtime',
        'perception_executable': 'perception',
    }
    args = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    common = {key: LaunchConfiguration(key) for key in ('profile', 'run_id')}
    extra = {'execution': tuple(DEFAULT_DRIVERS), 'recorder': ('output','recording_mode'),
             'task': ('planner_provider', 'chunk_provider', 'navigation_provider', 'navigation_config', 'navigation_backend'), 'perception': ('perception_provider',)}
    nodes = []
    for name, keys in extra.items():
        parameters = {**common, **{key: ParameterValue(LaunchConfiguration(key),value_type=str) if key=='recording_mode' else LaunchConfiguration(key) for key in keys}}
        nodes.append(Node(
            package=LaunchConfiguration('perception_package') if name=='perception' else 'bindu_runtime',
            executable=LaunchConfiguration('perception_executable') if name=='perception' else name,
            name=name, namespace=LaunchConfiguration('namespace'), parameters=[parameters], output='screen'))
    nodes.append(Node(package='bindu_runtime', executable='pi_client', name='pi_client',
                      namespace=LaunchConfiguration('namespace'), condition=IfCondition(LaunchConfiguration('pi_enabled')),
                      parameters=[{**common, 'pi_config': LaunchConfiguration('pi_config')}], output='screen'))
    return LaunchDescription(args+nodes)
