"""Sensor-only Orin mapping/localization. Real base adapter remains explicit."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from bindu_navigation.orin import load_site, site_errors, MOTION_BLOCK


def assemble(context):
    site_path = LaunchConfiguration('site').perform(context)
    phase = LaunchConfiguration('phase').perform(context)
    if phase not in ('mapping','localization'):
        raise ValueError(MOTION_BLOCK)
    site = load_site(site_path)
    errors = site_errors(site, phase)
    if errors: raise ValueError('\n'.join(errors))
    share = Path(get_package_share_directory('bindu_runtime'))
    common = {key: str(site[key]) for key in ('namespace','scan_topic','odom_topic','tf_topic','tf_static_topic')}
    common['mode'] = phase
    if site['route'] == '2d':
        common.update(navigation_robot=site_path, map=site['map']['yaml'] or '')
        filename = 'localization.launch.py'; nodes = []
    else:
        if phase == 'mapping' and Path(site['map']['prefix']+'.pcd').exists():
            raise ValueError('MAP_ALREADY_EXISTS: 使用新的map.prefix，禁止覆盖PCD')
        common.update(sensor_config=site_path, save_map=site['map']['prefix']+'.pcd',
                      pcd_map=site['map']['pcd'] or '',
                      grid_map=site['map']['yaml'] if phase=='localization' else '')
        filename = 'lio.launch.py'; nodes = []
        if phase == 'mapping':
            nodes.append(Node(package='bindu_navigation',executable='lio_observed_grid',namespace=site['namespace'],
                parameters=[{**site['grid'], 'map_frame':site['frames']['map'], 'max_age':site['checks']['max_age']}],
                remappings=[('/tf',site['tf_topic']),('/tf_static',site['tf_static_topic']),
                            ('navigation/scan',site['scan_topic'])],output='screen'))
    # Existing launch retains its simulation defaults; this entry always uses system time.
    nodes.insert(0, IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/filename)),
                                            launch_arguments=common.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('site'),DeclareLaunchArgument('phase'),
                              OpaqueFunction(function=assemble)])
