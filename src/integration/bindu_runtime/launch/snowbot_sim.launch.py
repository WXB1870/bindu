"""Compatibility entry for existing Snowbot simulation arguments."""
from bindu_contracts.resources import package_share
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from bindu_runtime.teleop_assembly import assemble

def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'runtime_mode': 'simulation', 'robot_config': '', 'namespace': 'bindu_snowbot_sim', 'run_id': uuid.uuid4().hex,
                'output': '/tmp/bindu-snowbot-runs', 'recording_mode': 'compact',
                'profile': str(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_sim.json'),
                'bimanual_config': str(package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_sim.json'),
                'device_backend': 'kinematic', 'side': 'left', 'initialize_tool_pose': 'false',
                'tool_input_source': 'vr',
                'description_enabled': 'true', 'tool_display_enabled': 'false',
                'pico_enabled': 'false', 'pico_config': str(package_share("snowbot_bringup")/'config/robots/snowbot/pico_snowbot_sim.json'),
                'single_arm_enabled':'true','recorder_enabled':'true',
                'command_enabled':'false','vendor_namespace':'/snowbot',
                'arm_receiver_node':'','servo_receiver_node':'','mid_receiver_node':'',
                'vr_enabled': 'false', 'input_timing_enabled':'false', 'host': '127.0.0.1', 'port': '8012',
                'cert_file': '', 'key_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, v in defaults.items()] + [OpaqueFunction(function=assemble)])
