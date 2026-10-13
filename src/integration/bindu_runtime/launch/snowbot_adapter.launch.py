"""Dedicated Snowbot endpoint. Read-only by default; no vendor process startup."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    profile = str(Path(get_package_share_directory('bindu_runtime'))/'config/snowbot_vendor_position.json')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bindu_snowbot'),
        DeclareLaunchArgument('vendor_namespace', default_value='/snowbot'),
        DeclareLaunchArgument('command_enabled', default_value='false'),
        DeclareLaunchArgument('simulation', default_value='false'),
        Node(package='bindu_runtime', executable='snowbot_adapter',
             namespace=LaunchConfiguration('namespace'), output='screen', parameters=[{
                 'profile': profile, 'vendor_namespace': LaunchConfiguration('vendor_namespace'),
                 'command_enabled': ParameterValue(LaunchConfiguration('command_enabled'), value_type=bool),
                 'simulation': ParameterValue(LaunchConfiguration('simulation'), value_type=bool)}]),
    ])
