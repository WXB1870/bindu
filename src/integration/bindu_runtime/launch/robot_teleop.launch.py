"""Robot-selected teleoperation. Validation finishes before any node starts."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory

from bindu_runtime.teleop_assembly import assemble

def generate_launch_description():
    share = Path(get_package_share_directory('bindu_runtime'))
    defaults = {'runtime_mode': 'simulation', 'robot_config': str(share/'config/robot_snowbot.json'),
                'namespace': 'bindu_teleop', 'run_id': uuid.uuid4().hex,
                'device_backend': 'kinematic', 'side': 'left', 'tool_input_source': 'tool_pose',
                'output': '/tmp/bindu-runs', 'recording_mode': 'compact',
                'description_enabled': 'false', 'tool_display_enabled': 'true',
                'vr_enabled': 'false', 'host': '127.0.0.1', 'port': '8012',
                'cert_file': '', 'key_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=assemble)])
