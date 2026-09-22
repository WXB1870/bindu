"""Actual SLAM Toolbox or map-server/AMCL; mutually exclusive TF authorities."""
import json
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def assemble(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    ns=value('namespace');mode=value('mode')
    robot=json.loads(Path(value('navigation_robot')).read_text())
    cfg=yaml.safe_load(Path(value('parameters')).read_text());f=robot['frames'];laser=robot['laser']
    remaps=[('/tf','tf'),('/tf_static','tf_static'),('map','navigation/map'),
            ('map_metadata','navigation/map_metadata'),('pose','localization/pose'),('amcl_pose','localization/pose')]
    def node(package,exe,name,params):
        return Node(package=package,executable=exe,name=name,namespace=ns,
                    parameters=[{'use_sim_time':False,**params}],remappings=remaps,output='screen')
    if mode=='mapping':
        params={**cfg['slam'],'odom_frame':f['odom'],'map_frame':f['map'],'base_frame':f['base'],
                'scan_topic':ns.rstrip('/')+'/navigation/scan','map_name':ns.rstrip('/')+'/navigation/map','min_laser_range':laser['min_range'],'max_laser_range':laser['max_range']}
        if value('pose_graph'):
            graph=Path(value('pose_graph')).resolve()
            if not Path(str(graph)+'.posegraph').is_file() or not Path(str(graph)+'.data').is_file():
                raise ValueError('SERIALIZED_POSE_GRAPH_REQUIRED')
            params.update(map_file_name=str(graph),map_start_at_dock=True)
        nodes=[node('slam_toolbox','async_slam_toolbox_node','slam_toolbox',params)]
        names=['slam_toolbox']
    elif mode=='localization':
        map_path=Path(value('map')).resolve()
        if not map_path.is_file():raise ValueError('SAVED_MAP_REQUIRED')
        params={**cfg['amcl'],'odom_frame_id':f['odom'],'global_frame_id':f['map'],'base_frame_id':f['base'],
                'scan_topic':'navigation/scan','map_topic':'navigation/map',
                'laser_min_range':laser['min_range'],'laser_max_range':laser['max_range']}
        nodes=[node('nav2_map_server','map_server','map_server',{'yaml_filename':str(map_path),'topic_name':'navigation/map','frame_id':f['map']}),
               node('nav2_amcl','amcl','amcl',params)]
        names=['map_server','amcl']
    else:raise ValueError('MODE_MUST_BE_MAPPING_OR_LOCALIZATION')
    nodes.append(node('nav2_lifecycle_manager','lifecycle_manager','localization_manager',
        {'autostart':True,'bond_timeout':2.,'node_names':names}))
    return nodes


def generate_launch_description():
    default=Path(get_package_share_directory('bindu_runtime'))/'config/navigation_localization.yaml'
    return LaunchDescription([DeclareLaunchArgument('namespace'),DeclareLaunchArgument('navigation_robot'),
        DeclareLaunchArgument('mode',default_value='mapping'),DeclareLaunchArgument('map',default_value=''),
        DeclareLaunchArgument('pose_graph',default_value=''),
        DeclareLaunchArgument('parameters',default_value=str(default)),OpaqueFunction(function=assemble)])
