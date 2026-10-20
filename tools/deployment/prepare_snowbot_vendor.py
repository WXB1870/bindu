#!/usr/bin/env python3
"""Prepare a separate controller copy from reviewed ZIP/118 sources; never run it."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import tempfile
import zipfile

SOURCE_SHA256 = '24170d47483dc25c391b64662d2ff41d945e3318dc201fd27c4529410ca625f3'
ASSETS = Path(__file__).resolve().parent/'snowbot_vendor'

# Pin the changed files and the execution kernels used by the 118 patch.
# The older ZIP/guard path below remains a distinct, incompatible source line.
SITE_118_FILES = {
    'dual_arm_control/include/controller/ArmController.h':
        '5311a62a2ae0669618a93fda1016eafc7962c7993ee88305eca48a875ca59a2e',
    'dual_arm_control/include/controller/TopicJointState.h':
        '5fa62daf483525680be606aadbe277b084349c000c12c4b2da8df757b0401b65',
    'dual_arm_control/include/dual_arm_control/msgPublish.h':
        '63d81b1b4508589a939ebcb3016e49f823b4bbb8f5e8547da60c7d5cbe0a2596',
    'dual_arm_control/include/robot_config/msgDefine.h':
        'fa38978db08dd9908ecfeb36002f520562f4f7dc91dd60c68dbd18d379c4a5f2',
    'dual_arm_control/src/controller/ArmController.cpp':
        'eafbd7e537bb8229ccfdd8ac9a68111df1aabc6d22f09e1128197735dd372598',
    'dual_arm_control/src/dual_arm_control/msgPublish.cpp':
        '47fd3557eaf12944fd917f91dc64eb4593a545e9d37ac42df5929419b77461dd',
    'robot_control/include/robot_control/msgPublisher.h':
        'b0bffc3a8b3dc039d5ba95a7f807ddcaa1c1cfb8e948cf91ef06c636e06fd20d',
    'robot_control/src/robot_control/msgPublisher.cpp':
        '81c37a8436d3e49d0729c79b8cb8b2926c997711631e532b980aa5fd509ffdd1',
    'servo_control/include/JntController/runinController.h':
        '684cd5cfff6f0dd425126bf22cd105d6189a64503a6c0bc39759dc41fe865af4',
    'servo_control/include/ros_controller/rosLoopMsgDefine.h':
        '631e6fc13c5f9f0c9a767a99b458d8bf2b65788d55a9d900f82ca1615628f87a',
    'servo_control/src/JntController/runinController.cpp':
        '790efaa87bb10f0c7d6d6370c88c4332d18268c2a0f6e4394dd443d14778f261',
    'servo_control/src/ros_controller/rosLoop.cpp':
        '2539801ceed2b10ab695e69da08591772dc2ef2327fc97226e513b4392d2ea38',
}


SITE_118_FEEDBACK_FILES = {
    'ctl_msgs/msg/RetInformation.msg':
        '7925166be4a6ec7e008fe1bf6893e3f24743ec2a3f07e41dcc789eb9d24cc5d3',
    'servo_control/include/servo_control/ethercat_driver.h':
        '2d47b3224261ef21850af6093fefd4019c5cc955e2f8a609711d039953038485',
    'servo_control/include/servo_control/transmit.h':
        'cde444e060f532d21ebc94aafa8a49366d140bbfd0d5009858dd977bcabd0c0e',
    'servo_control/src/servo_control/ethercat_driver.cpp':
        'bf4edd5ca3da8892bde7ed3d2dbc460415d1798f125665cafb4a14db8ffe963b',
    'servo_control/src/servo_control/transmit.cpp':
        '6476366b45bb1ead0d977a7b7fd0dfab70d40d020633db60ef010908d2dad548',
}

SITE_118_GUARD_FILES = {
    'servo_control/include/ros_controller/rosLoop.h':
        'c0ab7fdf401c60fb2afe056726da41abd7a1812200f867b18acf4ca280761e0c',
    'ctl_msgs/msg/JointPositionCommand.msg':
        '3e5bd58056f00a0450d1f0154a40a4e0c7d990d5fe49155ee6a9dd013b9f6cb4',
    'ctl_msgs/msg/MidNodeCtlMsgSend.msg':
        '2a18e59b4bd655dcb02e67ddb8e7449aa4f956ee8e4f12a298049db5aeefa887',
    'ctl_msgs/msg/CtlInformation.msg':
        '0a8769ed34ca9820123e046195cfadbc575e78a9dcebd0828d33ea30cc00be8d',
}


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def prepare(archive, output):
    archive, output = Path(archive).resolve(), Path(output).absolute()
    if digest(archive) != SOURCE_SHA256:
        raise ValueError('SNOWBOT_SOURCE_VERSION_MISMATCH')
    if output.exists() or output.is_symlink():
        raise ValueError('SNOWBOT_OUTPUT_EXISTS')
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.snowbot-', dir=output.parent))
    try:
        with zipfile.ZipFile(archive) as bundle:
            for member in bundle.infolist():
                path = PurePosixPath(member.filename)
                if (path.is_absolute() or '..' in path.parts or not path.parts or
                        path.parts[0] != 'src' or '\\' in member.filename or
                        stat.S_ISLNK(member.external_attr >> 16)):
                    raise ValueError('SNOWBOT_UNSAFE_ARCHIVE_PATH')
                destination = stage.joinpath(*path.parts)
                if member.is_dir():
                    destination.mkdir(parents=True, exist_ok=True)
                else:
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    with bundle.open(member) as source, destination.open('xb') as target:
                        shutil.copyfileobj(source, target)
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                        '-i', str(ASSETS/'bindu_guard.patch')], cwd=stage/'src',
                       check=True, capture_output=True, text=True)
        shutil.copyfile(ASSETS/'bindu_watchdog.h',
                        stage/'src/servo_control/include/servo_control/bindu_watchdog.h')
        manifest = {'schema': 1, 'source_sha256': SOURCE_SHA256,
                    'patch_sha256': digest(ASSETS/'bindu_guard.patch'),
                    'watchdog_sha256': digest(ASSETS/'bindu_watchdog.h'),
                    'guard_default': False, 'hardware_started': False}
        (stage/'bindu_vendor_build.json').write_text(json.dumps(manifest, indent=2)+'\n')
        stage.rename(output)
    except BaseException:
        shutil.rmtree(stage)
        raise
    return output


def prepare_118(source, output, *, feedback=False, guard=False):
    """Copy reviewed 118 sources; optionally extend measured PDO feedback after position fixes."""
    source, output = Path(source).resolve(), Path(output).absolute()
    feedback = feedback or guard
    if not source.is_dir() or not SITE_118_FILES:
        raise ValueError('SNOWBOT_118_SOURCE_REQUIRED')
    if (output.resolve() == source or source in output.resolve().parents or
            output.resolve() in source.parents):
        raise ValueError('SNOWBOT_SOURCE_OUTPUT_OVERLAP')
    if output.exists() or output.is_symlink():
        raise ValueError('SNOWBOT_OUTPUT_EXISTS')
    if any(path.is_symlink() for path in source.rglob('*')):
        raise ValueError('SNOWBOT_118_SOURCE_SYMLINK')
    expected_files = dict(SITE_118_FILES, **(SITE_118_FEEDBACK_FILES if feedback else {}))
    if guard:
        expected_files.update(SITE_118_GUARD_FILES)
    for relative, expected in expected_files.items():
        path = source/relative
        if path.is_symlink() or not path.is_file() or digest(path) != expected:
            raise ValueError('SNOWBOT_118_SOURCE_VERSION_MISMATCH:'+relative)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.snowbot-118-', dir=output.parent))
    try:
        shutil.copytree(source, stage/'src', symlinks=True)
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                        '-i', str(ASSETS/'position_118.patch')], cwd=stage/'src',
                       check=True, capture_output=True, text=True)
        for package in ('dual_arm_control', 'servo_control'):
            shutil.copyfile(ASSETS/'position_contract.h',
                            stage/'src'/package/'include/bindu_position_contract.h')
        if feedback:
            subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                            '-i', str(ASSETS/'feedback_118.patch')], cwd=stage/'src',
                           check=True, capture_output=True, text=True)
            shutil.copyfile(ASSETS/'feedback_snapshot.h',
                            stage/'src/servo_control/include/servo_control/bindu_feedback_snapshot.h')
        if guard:
            subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                            '-i', str(ASSETS/'guard_118.patch')], cwd=stage/'src',
                           check=True, capture_output=True, text=True)
            for package in ('servo_control', 'robot_control', 'dual_arm_control'):
                shutil.copyfile(ASSETS/'guard_118.h',
                                stage/'src'/package/'include/bindu_guard_118.h')
        manifest = dict(schema=1, source='snowbot_118_native_position',
                        source_files=expected_files,
                        patch_sha256=digest(ASSETS/'position_118.patch'),
                        helper_sha256=digest(ASSETS/'position_contract.h'),
                        hardware_started=False, full_driver_build_verified=False)
        if feedback:
            manifest.update(feedback_patch_sha256=digest(ASSETS/'feedback_118.patch'),
                            feedback_helper_sha256=digest(ASSETS/'feedback_snapshot.h'))
        if guard:
            manifest.update(guard_patch_sha256=digest(ASSETS/'guard_118.patch'),
                            guard_helper_sha256=digest(ASSETS/'guard_118.h'),
                            guard_default=False, native_position_guard_version=1)
        (stage/'bindu_vendor_build.json').write_text(json.dumps(manifest, indent=2)+'\n')
        stage.rename(output)
    except BaseException:
        shutil.rmtree(stage)
        raise
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--site-118', action='store_true',
                        help='输入为本机118源码根目录；生成独立副本，只修复首帧/使能/计时，不包含保护补丁')
    parser.add_argument('--feedback', action='store_true',
                        help='与--site-118合用，增加20轴真实反馈及PDO采样元数据，不启动设备')
    parser.add_argument('--guard', action='store_true',
                        help='与--site-118合用，包含反馈及原生位置期限保护；保护模式须现场显式选择')
    args = parser.parse_args()
    if args.feedback and not args.site_118:
        parser.error('--feedback requires --site-118')
    if args.guard and not args.site_118:
        parser.error('--guard requires --site-118')
    print(prepare_118(args.archive, args.output, feedback=args.feedback, guard=args.guard) if args.site_118
          else prepare(args.archive, args.output))


if __name__ == '__main__':
    main()
