"""Composition root: independently choose body, hand, and base adapters."""
import json
import math
from bindu_runtime.plugins import load_provider
from bindu_hardware.robot_io.system import RobotIO

DEFAULT_DRIVERS = {
    'joint_driver_provider': 'bindu_hardware.drivers.simulated_joints:SimJointDriver',
    'hand_driver_provider': 'bindu_hardware.drivers.simulated_hand:SimHandDriver',
    'base_driver_provider': 'bindu_hardware.drivers.simulated_base:SimBaseDriver',
}


def assemble_devices(node, profile):
    node.declare_parameter('device_backend', 'kinematic')
    backend = node.get_parameter('device_backend').value
    node.declare_parameter('kinematic_initial_positions', '{}')
    initial = json.loads(node.get_parameter('kinematic_initial_positions').value)
    if (not isinstance(initial,dict) or set(initial)-set(profile.limits) or
            any(not isinstance(q,(int,float)) or not math.isfinite(q) or
                not profile.limits[n][0] <= q <= profile.limits[n][1] for n,q in initial.items())):
        raise ValueError('INVALID_KINEMATIC_INITIAL_POSITIONS')
    if initial and backend != 'kinematic':
        raise ValueError('INITIAL_POSE_ONLY_FOR_KINEMATIC_FIXTURE')
    if backend == 'external_simulation':
        from .simulation_devices import SimulationDevices, PhysicsJoints, PhysicsBase
        transport = SimulationDevices(node, profile)
        joints = {group: PhysicsJoints(transport, group) for group in profile.groups}
        base = PhysicsBase(transport)
        return RobotIO(profile, joints, base), {**joints, 'base': base}
    if backend != 'kinematic':
        raise ValueError('UNKNOWN_DEVICE_BACKEND')
    for name, default in DEFAULT_DRIVERS.items():
        node.declare_parameter(name, default)
    joints = {}
    for group, names in profile.groups.items():
        parameter = 'hand_driver_provider' if group == 'hand' else 'joint_driver_provider'
        provider = node.get_parameter(parameter).value
        if initial:
            if provider != DEFAULT_DRIVERS['joint_driver_provider']:
                raise ValueError('INITIAL_POSE_REQUIRES_SIM_JOINT_DRIVER')
            joints[group] = load_provider(provider, names, profile.max_speed,
                                           {n:initial.get(n,0.) for n in names})
        else:
            joints[group] = load_provider(provider, names, profile.max_speed)
    base = load_provider(node.get_parameter('base_driver_provider').value)
    return RobotIO(profile, joints, base), {**joints, 'base': base}


def inject_simulated_fault(devices, request):
    # Optional group prefix makes one-device failure testable, e.g. hand:feedback_loss.
    if ':' in request:
        name, fault = request.split(':', 1)
        if name not in devices:
            return False
        targets = [devices[name]]
    else:
        fault, targets = request, list(devices.values())
    if fault not in ('', 'reject', 'feedback_loss') or not all(hasattr(d, 'inject_fault') for d in targets):
        return False
    for driver in targets:
        driver.inject_fault(fault)
    return True
