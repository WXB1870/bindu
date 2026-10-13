#!/usr/bin/env python3
"""Compile patched vendor units and exercise the driver with a fake PDO bus."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ros-prefix', required=True, type=Path)
    parser.add_argument('--messages-prefix', required=True, type=Path)
    args = parser.parse_args()
    src, output = args.source.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    servo = src/'servo_control'
    includes = [servo/'include', servo/'thirdparty/soem/soem',
                servo/'thirdparty/soem/osal', servo/'thirdparty/soem/osal/linux',
                servo/'thirdparty/soem/oshw/linux']
    flags = ['-std=c++17', '-O1', '-pthread']+['-I'+str(p) for p in includes]
    def run(name, command):
        with (output/(name+'.log')).open('w') as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    exe = output/'fake-pdo'
    run('compile-fake-pdo', ['g++', *flags, '-ffunction-sections', '-fdata-sections',
                           '-Wl,--gc-sections', str(root/'tests/test_snowbot_vendor.cpp'),
                           str(servo/'src/servo_control/ethercat_driver.cpp'), '-o', str(exe)])
    run('fake-pdo', [str(exe)])
    ros_includes = [args.ros_prefix/'include', *sorted((args.ros_prefix/'include').glob('*')),
                    *sorted((args.messages_prefix/'include').glob('*')), Path('/usr/include/eigen3')]
    ros_flags = flags+['-I'+str(p) for p in ros_includes if p.is_dir()]
    for name, source in [('servo-loop', servo/'src/ros_controller/rosLoop.cpp'),
                         ('transmit', servo/'src/servo_control/transmit.cpp'),
                         ('upper-source', src/'robot_control/src/robot_control/msgPublisher.cpp')]:
        run(name, ['g++', *ros_flags, '-I'+str(src/'robot_control/include'),
                   '-fsyntax-only', str(source)])
    (output/'results.json').write_text(json.dumps(dict(fake_pdo=True, compiled_units=3,
        hardware_started=False, full_vendor_build=False), indent=2)+'\n')
    print('Fake PDO cycle and 3 patched ROS translation units passed; no hardware started')


if __name__ == '__main__':
    main()
