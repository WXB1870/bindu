"""Compatibility entry for existing Snowbot simulation arguments."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from bindu_runtime.teleop_assembly import assemble

def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'robot_config': '', 'namespace': 'bindu_snowbot_sim', 'run_id': uuid.uuid4().hex,
                'output': '/tmp/bindu-snowbot-runs', 'recording_mode': 'compact',
                'profile': str(share / 'config/snowbot_sim.json'),
                'bimanual_config': str(share / 'config/bimanual_snowbot_sim.json'),
                'device_backend': 'kinematic', 'side': 'left', 'initialize_tool_pose': 'false',
                'tool_input_source': 'vr',
                'description_enabled': 'true', 'tool_display_enabled': 'false',
                'vr_enabled': 'false', 'host': '127.0.0.1', 'port': '8012',
                'cert_file': '', 'key_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, v in defaults.items()] + [OpaqueFunction(function=assemble)])
