"""Embodiment-neutral physical navigation assembly; simulator is external."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args=[DeclareLaunchArgument(k) for k in ('namespace','profile','navigation_robot','navigation_config','output','run_id','identity_bridge')]
    args.append(DeclareLaunchArgument('require_scan',default_value='false'))
    args.append(DeclareLaunchArgument('recording_mode',default_value='normal'))
    common={k:LaunchConfiguration(k) for k in ('profile','run_id')}
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
    return LaunchDescription(args+nodes)
