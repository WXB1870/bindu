"""Historical FAST-LIO + seeded 3D ICP; no device driver or motion is started."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def assemble(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace = value('namespace')
    sensor = json.loads(Path(value('sensor_config')).read_text())
    parameters = yaml.safe_load(Path(value('parameters')).read_text())
    mode = value('mode')
    if mode not in ('mapping', 'localization'):
        raise ValueError('MODE_MUST_BE_MAPPING_OR_LOCALIZATION')
    if sensor['input_type'] not in ('timed_cloud', 'livox'):
        raise ValueError('LIO_INPUT_TYPE_INVALID')
    frames = sensor['frames']
    transforms = {name: sensor[name] for name in ('imu_from_lidar', 'imu_from_base')}
    frame_parameters = {name + '_frame': frames[name] for name in ('odom', 'base', 'lidar', 'imu')}
    use_sim_time = value('use_sim_time').lower() == 'true'
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('lio/imu', sensor['imu_topic']),
              ('lio/odom', 'navigation/odom')]
    if sensor['input_type'] == 'timed_cloud':
        remaps.append(('lio/points', sensor['points_topic']))
    nodes = [Node(
        package='bindu_lio', executable='fast_lio', namespace=namespace, output='screen',
        parameters=[{**parameters['lio'], **frame_parameters, **transforms,
                     'save_map_path': value('save_map'), 'use_sim_time': use_sim_time}],
        remappings=remaps)]
    if sensor['input_type'] == 'livox':
        nodes.append(Node(
            package='bindu_lio', executable='livox_input', namespace=namespace, output='screen',
            parameters=[{'lidar_frame': frames['lidar'], 'scan_lines': sensor['scan_lines'],
                         'use_sim_time': use_sim_time}],
            remappings=[('livox/lidar', sensor['points_topic'])]))
    if 'scan_projection' in sensor:
        nodes.append(Node(
            package='bindu_lio', executable='lio_scan', namespace=namespace, output='screen',
            parameters=[{**sensor['scan_projection'], **transforms,
                         **{name + '_frame': frames[name] for name in ('odom', 'base', 'lidar')},
                         'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('lio/odom', 'navigation/odom')]))
    if mode == 'localization':
        path = Path(value('pcd_map')).resolve()
        if not path.is_file():
            raise ValueError('PCD_MAP_REQUIRED')
        nodes.append(Node(
            package='bindu_lio', executable='lio_localization', namespace=namespace, output='screen',
            parameters=[{**parameters['registration'], 'map_path': str(path),
                         'map_frame': frames['map'], 'odom_frame': frames['odom'],
                         'base_frame': frames['base'], 'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('lio/odom', 'navigation/odom')]))
    if value('grid_map'):
        if mode != 'localization':
            raise ValueError('GRID_MAP_ONLY_WITH_LOCALIZATION')
        grid_map = Path(value('grid_map')).resolve()
        if not grid_map.is_file():
            raise ValueError('VERIFIED_GRID_MAP_REQUIRED')
        nodes.extend([
            Node(package='nav2_map_server', executable='map_server', name='lio_map_server',
                 namespace=namespace, output='screen', parameters=[{
                     'yaml_filename': str(grid_map), 'topic_name': 'navigation/map',
                     'frame_id': frames['map'], 'use_sim_time': use_sim_time}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lio_map_manager', namespace=namespace, output='screen', parameters=[{
                     'autostart': True, 'node_names': ['lio_map_server'], 'use_sim_time': use_sim_time}]),
        ])
    return nodes


def generate_launch_description():
    default = Path(get_package_share_directory('bindu_runtime')) / 'config/navigation_lio.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('namespace'), DeclareLaunchArgument('sensor_config'),
        DeclareLaunchArgument('parameters', default_value=str(default)),
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('pcd_map', default_value=''),
        DeclareLaunchArgument('grid_map', default_value=''),
        DeclareLaunchArgument('save_map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=assemble),
    ])
