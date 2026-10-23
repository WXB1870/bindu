"""Read-only Linux controller inventory before replacing or building a deployment."""
import json
import os
from pathlib import Path


CONTROLLERS = frozenset((
    'servo_node', 'dual_arm_control_node', 'robot_control_node', 'robot_upperMonitor',
    'snowbot_adapter', 'pico_control', 'teleop_tool', 'execution', 'teleop',
))
MODULES = {
    'bindu_runtime.snowbot_adapter': 'snowbot_adapter',
    'bindu_runtime.pico_node': 'pico_control',
    'bindu_runtime.bimanual_tool_node': 'teleop_tool',
    'bindu_runtime.execution_node': 'execution',
    'bindu_runtime.teleop_node': 'teleop',
    'tools.teleop.run_vr': 'run_vr',
}


def controller_entries(argv):
    # Linux script cmdlines start with the Python interpreter. Exact token
    # matching also covers ros2 run wrappers, without exposing full arguments.
    entries = {os.path.basename(arg) for arg in argv
               if os.path.basename(arg) in CONTROLLERS}
    for index, arg in enumerate(argv[:-1]):
        if arg == '-m' and argv[index+1] in MODULES:
            entries.add(MODULES[argv[index+1]])
    return sorted(entries)


def scan_controllers(proc_root=Path('/proc')):
    proc_root = Path(proc_root)
    if not proc_root.is_dir():
        raise ValueError('DEPLOYMENT_PROCESS_SCAN_UNAVAILABLE')
    running, unreadable = [], []
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            argv = [os.fsdecode(value) for value in
                    (process/'cmdline').read_bytes().split(b'\0') if value]
        except (FileNotFoundError, ProcessLookupError):
            continue  # A process that exited during the snapshot.
        except OSError:
            unreadable.append(int(process.name))
            continue
        entries = controller_entries(argv)
        if entries:
            running.append(dict(pid=int(process.name), entries=entries))
    return dict(controllers=sorted(running, key=lambda item: item['pid']),
                unreadable_pids=sorted(unreadable))


def require_stopped(proc_root=Path('/proc')):
    report = scan_controllers(proc_root)
    if report['unreadable_pids']:
        raise ValueError('DEPLOYMENT_PROCESS_STATE_UNKNOWN:'+json.dumps(report))
    if report['controllers']:
        raise ValueError('DEPLOYMENT_CONTROLLERS_RUNNING:'+json.dumps(report))
    return report


if __name__ == '__main__':
    try:
        print(json.dumps(require_stopped()))
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc))
