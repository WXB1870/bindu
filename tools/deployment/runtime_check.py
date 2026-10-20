"""Offline checks shared by the unified VR and minimal real deployment tools."""
import importlib
import json
from pathlib import Path

from tools import ROOT


def check_installed_configs(paths, share):
    """Site overrides are independent; package configs must match this checkout."""
    source_share = ROOT/'src/integration/bindu_runtime'
    share = Path(share).resolve()
    for value in paths:
        if not value:
            continue
        installed = Path(value).resolve()
        if installed.is_relative_to(share):
            source = source_share/installed.relative_to(share)
            if not source.is_file() or source.read_bytes() != installed.read_bytes():
                raise ValueError('安装配置与源码不同，请重建：'+str(source))


def check_runtime(args, robot):
    """Load native libraries/typesupport and construct QP; no targets or device IO."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.type_support import check_for_type_support
    from bindu_kinematics.bimanual import BimanualQP

    check_installed_configs(
        [args['robot_config'], robot.profile_path, robot.tool_config,
         args.get('pico_config'), args.get('gripper_config')],
        get_package_share_directory('bindu_runtime'))
    modules = [
        ('bindu_interfaces.msg', ('ExecutionState', 'VRInput', 'TeleopModeState', 'GripperState')),
        ('bindu_interfaces.action', ('SelectTeleopMode', 'TeleopSession')),
        ('bindu_interfaces.srv', ('GripperControl',)),
    ]
    if args['runtime_mode'] == 'real':
        modules.append(('snowbot_ctl_msgs.msg', ('JointPositionCommand', 'RetInformation')))
    for module, names in modules:
        package = importlib.import_module(module)
        for name in names:
            check_for_type_support(getattr(package, name))
    for module in ('pinocchio', 'casadi', 'vuer', 'bindu_execution.interpolation'):
        importlib.import_module(module)
    tool = json.loads(robot.tool_config.read_text())
    tool['urdf'] = str(robot.urdf)
    solver = BimanualQP(tool, robot.profile)
    solver.close()
