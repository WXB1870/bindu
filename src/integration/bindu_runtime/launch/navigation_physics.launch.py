"""Embodiment-neutral physical navigation assembly; simulator is external."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution
from bindu_contracts.runtime_mode import resolve_mode


def validate_mode(context):
    if resolve_mode(LaunchConfiguration('runtime_mode').perform(context)) != 'simulation':
        raise ValueError('NAVIGATION_REAL_BACKEND_NOT_IMPLEMENTED')
    return []


def generate_launch_description():
    args=[DeclareLaunchArgument(k) for k in ('namespace','profile','navigation_robot','navigation_config','output','run_id')]
    args.append(DeclareLaunchArgument('identity_bridge',default_value=PathJoinSubstitution([
        FindPackagePrefix('bindu_navigation'),'lib','bindu_navigation','nav2_velocity_identity'])))
    args.append(DeclareLaunchArgument('require_scan',default_value='false'))
    args.append(DeclareLaunchArgument('recording_mode',default_value='compact'))
    args.append(DeclareLaunchArgument('runtime_mode',default_value='simulation'))
    common={k:LaunchConfiguration(k) for k in ('profile','run_id','runtime_mode')}
    nodes=[]
    for name,extra in [('execution',{'device_backend':'external_simulation'}),
                       ('recorder',{'output':LaunchConfiguration('output'), 'recording_mode':ParameterValue(LaunchConfiguration('recording_mode'),value_type=str)}),
                       ('task',{'navigation_provider':'bindu_runtime.navigation:Nav2Navigation',
                                'navigation_config':LaunchConfiguration('navigation_config')})]:
        nodes.append(Node(package='bindu_runtime',executable=name,namespace=LaunchConfiguration('namespace'),
            parameters=[{**common,**extra}],output='screen'))
    nodes.append(Node(package='bindu_runtime',executable='nav2_session',namespace=LaunchConfiguration('namespace'),
        parameters=[{**{k:LaunchConfiguration(k) for k in ('navigation_robot','navigation_config','output','identity_bridge')},
            'require_scan':ParameterValue(LaunchConfiguration('require_scan'),value_type=bool)}],
        remappings=[('/tf','tf'),('/tf_static','tf_static')],output='screen'))
    return LaunchDescription(args+[OpaqueFunction(function=validate_mode)]+nodes)
