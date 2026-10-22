"""Offline session timing: native planning/import/commit, pipes replace ROS/DDS.

No robot, ROS node, socket, or control service is accessed. The native probe
uses the production trajectory importer and receipt queue, with a normal-priority
1 ms commit poll and 2/50 ms receipt dispatch. Results are local measurements,
not a hardware-safe lead recommendation or an end-to-end teleoperation test.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace

from tools import ROOT


def clock_ns():
    # Cross-process origin is calibrated below, never assumed on macOS.
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def calibrate(process):
    trials = []
    for _ in range(8):
        before = clock_ns()
        process.stdin.write('PING\n')
        process.stdin.flush()
        answer = process.stdout.readline().split()
        after = clock_ns()
        if len(answer) != 2 or answer[0] != 'CLOCK':
            raise RuntimeError('PROBE_CLOCK_CALIBRATION')
        trials.append((after-before, int(answer[1])-(before+after)//2))
    round_trip, offset = min(trials)
    return offset, round_trip/2


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values)-1, math.ceil(fraction*len(values))-1)]


def summary(values):
    return {name: percentile(values, fraction) for name, fraction in
            (('p50', .5), ('p95', .95), ('p99', .99), ('max', 1.))}


def encode(metadata, wire):
    values = [metadata['sequence'], metadata['operation'], metadata['source_stamp_ns'],
              metadata['expires_at_ns'], wire['effective_at_ns'], len(wire['duration'])]
    for key in ('max_velocity', 'max_acceleration', 'max_jerk'):
        values.extend(wire[key])
    offset = 0
    for piece in range(len(wire['duration'])):
        values.extend(wire[key][piece] for key in ('piece_start', 'duration', 'source_duration', 'source_elapsed'))
        for count in wire['control_count'][piece*16:(piece+1)*16]:
            values.append(count)
            values.extend(wire['controls'][offset:offset+count])
            offset += count
    return ' '.join(map(str, values))+'\n'


def compare(binary, lead, receipt_ms, samples, warmup, stress, delay, target_period_ms):
    from bindu_contracts.profile import Profile
    from bindu_runtime.snowbot_position_session import SessionTrajectory
    from bindu_hardware.snowbot_session import CONTROL_NAMES, SLOTS
    profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
    planner = SessionTrajectory(profile, lead_ms=lead)
    positions = [0.]*20
    positions[1], positions[9], positions[18], positions[19] = .2, -.2, .12, -.08
    observation = SimpleNamespace(position=positions, speed=[0.]*20)
    process = subprocess.Popen([str(binary), str(receipt_ms), str(stress), str(delay)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    records = []
    try:
        ready = process.stdout.readline().split()
        if len(ready) != 2 or ready[0] != 'READY':
            raise RuntimeError('PROBE_CLOCK_DOMAIN_OR_STARTUP')
        offset, uncertainty = calibrate(process)
        next_target = clock_ns()
        for index in range(warmup+samples):
            wait_ns = next_target-clock_ns()
            if wait_ns > 0:
                time.sleep(wait_ns*1e-9)
            started = clock_ns()
            next_target = started+round(target_period_ms*1e6)
            native_started = started+offset
            metadata = dict(sequence=index+1, operation=1 if planner.segment is None else 0,
                            source_stamp_ns=native_started, expires_at_ns=native_started+200_000_000)
            goal = {name: positions[SLOTS.index(name)] for name in CONTROL_NAMES}
            goal['arm_l_joint_1'] += .005*math.sin(index*.11)
            goal['arm_r_joint_1'] += .003*math.sin(index*.17)
            goal['body_joint_1'] += .002*math.sin(index*.07)
            proposal = planner.prepare(metadata, goal, observation, native_started*1e-9)
            planned = clock_ns()
            payload = encode(metadata, proposal.wire)
            encoded = clock_ns()
            process.stdin.write(payload)
            process.stdin.flush()
            answer = process.stdout.readline().split()
            received = clock_ns()
            if len(answer) != 6:
                raise RuntimeError('PROBE_RESPONSE:'+repr(answer))
            accepted = answer[0] == '1'
            if accepted:
                planner.commit(proposal)
            else:
                # Benchmark recovery uses a fresh at-rest takeover on the next
                # sample; it does not claim the production reconnect is automatic.
                planner = SessionTrajectory(profile, lead_ms=lead)
            if index >= warmup:
                records.append(dict(accepted=accepted, reason=answer[5], planning_ms=(planned-started)*1e-6,
                    encoding_ms=(encoded-planned)*1e-6, decision_ms=float(answer[1]),
                    receipt_ms=(received-started)*1e-6, native_receipt_ms=float(answer[2]),
                    remaining_ms=float(answer[3]), import_and_poll_ms=float(answer[4])))
        final_offset, final_uncertainty = calibrate(process)
        clock_drift_ms = abs(final_offset-offset)*1e-6
        if clock_drift_ms > 1.:
            raise RuntimeError('PROBE_CLOCK_DRIFT_EXCEEDS_1MS')
        process.stdin.write('QUIT\n')
        process.stdin.flush()
        process.wait(timeout=10)
        if process.returncode:
            raise RuntimeError(process.stderr.read())
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close()
    return dict(lead_ms=lead, receipt_period_ms=receipt_ms, samples=samples,
                clock_uncertainty_ms=max(uncertainty, final_uncertainty)*1e-6, clock_drift_ms=clock_drift_ms,
                rejected=sum(not row['accepted'] for row in records),
                statistics={key: summary([row[key] for row in records]) for key in
                    ('planning_ms', 'encoding_ms', 'decision_ms', 'receipt_ms', 'import_and_poll_ms')},
                min_remaining_ms=min(row['remaining_ms'] for row in records), records=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-root', type=Path, required=True, help='Local Snowbot maintenance repository')
    parser.add_argument('--vendor-source', type=Path, required=True, help='Locally prepared vendor src; never started')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=300)
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--leads-ms', default='60,30,20,15,10,5')
    parser.add_argument('--stress-threads', type=int, default=0, choices=range(5))
    parser.add_argument('--import-delay-ms', type=int, default=0, choices=range(51))
    parser.add_argument('--target-period-ms', type=float, default=20.,
                        help='Minimum target interval; 20 ms matches current executor, 0 tests saturation')
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000 or not 0 <= args.warmup <= 1000:
        parser.error('samples/warmup outside bounded offline range')
    if not math.isfinite(args.target_period_ms) or not 0 <= args.target_period_ms <= 100.:
        parser.error('target period outside 0..100 ms')
    if args.output.exists():
        parser.error('output must be a fresh directory')
    for package in ROOT.joinpath('src').glob('*/*/package.xml'):
        sys.path.insert(0, str(package.parent))
    native = ROOT/'src/control/bindu_execution/native'
    vendor = args.vendor_source.resolve()/'servo_control/include'
    source = args.vendor_root.resolve()/'tests/native/benchmark_position_latency.cpp'
    compiler = shutil.which('c++')
    if not compiler or not source.is_file():
        parser.error('local compiler or native probe missing')
    leads = [float(value) for value in args.leads_ms.split(',')]
    from bindu_runtime.snowbot_position_session import SessionTrajectory
    from bindu_contracts.profile import Profile
    profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
    for lead in leads:
        SessionTrajectory(profile, lead_ms=lead)
    args.output.mkdir(parents=True)
    binary = args.output/'latency-probe'
    command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread',
               '-I'+str(vendor), '-I'+str(vendor/'bindu_execution_native'), str(source), str(native/'interpolation.cpp'),
               str(native/'adaptive_stream.cpp'), '-o', str(binary)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    cases = [(60., 50), *((lead, 2) for lead in leads)]
    results = []
    for lead, receipt in cases:
        result = compare(binary, lead, receipt, args.samples, args.warmup, args.stress_threads,
                         args.import_delay_ms, args.target_period_ms)
        results.append(result)
        print(json.dumps({key: value for key, value in result.items() if key != 'records'}), flush=True)
    paths = [Path(__file__).resolve(), source, vendor/'position_trajectory.h', vendor/'position_receipt.h',
             ROOT/'src/integration/bindu_runtime/bindu_runtime/snowbot_position_session.py',
             *(native/name for name in ('interpolation.cpp', 'adaptive_stream.cpp', 'curve_math.hpp',
                                       'interpolation.hpp', 'adaptive_stream.hpp'))]
    report = dict(schema=1, platform=platform.platform(), python=sys.version,
        real_ros_dds=False, robot_connected=False, hardware_commands_sent=False,
        native_commit_poll_ms=1, fast_receipt_poll_ms=2, baseline_receipt_poll_ms=50,
        stress_threads=args.stress_threads, import_delay_ms=args.import_delay_ms,
        target_period_ms=args.target_period_ms,
        limits=dict(velocity=.25, acceleration=.5, jerk=162., source_ttl_ms=200, commit_reserve_ms=2),
        source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, results=results)
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
