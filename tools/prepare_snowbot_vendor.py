#!/usr/bin/env python3
"""Prepare a patched vendor workspace from the exact reviewed ZIP; never run it."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(prepare(args.archive, args.output))


if __name__ == '__main__':
    main()
