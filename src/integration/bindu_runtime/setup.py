from setuptools import setup, find_packages
from glob import glob
from pathlib import Path

# Preserve the source hierarchy in package share; test fixtures stay in tests/.
config_files = sorted(Path('config').rglob('*.json')) + sorted(Path('config').rglob('*.yaml'))
config_data = [(f'share/bindu_runtime/{folder}', [str(p) for p in config_files if p.parent == folder])
               for folder in sorted({p.parent for p in config_files})]

setup(name='bindu_runtime', version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages',['resource/bindu_runtime']),('share/bindu_runtime',['package.xml']),*config_data,('share/bindu_runtime/launch',glob('launch/*.py'))], install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, entry_points={'console_scripts':['snowbot_position_sim = bindu_runtime.snowbot_position_sim:main', 'snowbot_adapter = bindu_runtime.snowbot_adapter:main', 'check_robot = bindu_runtime.robot_setup:main', 'nav2_session = bindu_runtime.nav2_session:main', 'description_state = bindu_runtime.description_node:main', 'execution = bindu_runtime.execution_node:main', 'perception = bindu_runtime.perception_node:main', 'task = bindu_runtime.task_node:main', 'recorder = bindu_runtime.recorder_node:main', 'pi_client = bindu_runtime.pi_node:main', 'teleop = bindu_runtime.teleop_node:main', 'teleop_mode = bindu_runtime.teleop_mode_node:main', 'teleop_tool = bindu_runtime.bimanual_tool_node:main', 'teleop_display = bindu_runtime.teleop_display_node:main', 'vr_input = bindu_runtime.vr_input_node:main', 'pico_control = bindu_runtime.pico_node:main', 'soem_joint_feedback = bindu_runtime.soem_joint_feedback:main']})
