"""Load a teleoperation-only deployment recipe; keep existing robot launch owners."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from bindu_runtime.teleop_deployment import load_deployment


def assemble(context):
    share = Path(get_package_share_directory('bindu_runtime'))
    path = Path(LaunchConfiguration('config').perform(context))
    launch_file, arguments = load_deployment(path, share)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/launch_file)),
                                     launch_arguments=arguments.items())]


def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/deployments/teleop_robot_deployment.json')),
        OpaqueFunction(function=assemble)])
