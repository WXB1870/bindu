"""Legacy launch path; maintained by snowbot_bringup."""
from pathlib import Path
import runpy
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    path = Path(get_package_share_directory("snowbot_bringup"))/"launch/snowbot_adapter.launch.py"
    return runpy.run_path(str(path))["generate_launch_description"]()
