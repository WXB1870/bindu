"""Offline matched-input old/new reference comparison; no ROS or device access.

The deterministic clock advances by 1 ms. Actual generation wall time is rounded
up to that clock and injected compute delay is added. Late commits are rejected
by the production queue. Old ACK pacing waits 2 ms after each commit and then
uses the latest target; receipt latency here is an explicit scheduling model.
These results do not measure DDS, physical tracking, or real-time scheduling.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

from tools import ROOT


def statistics(values):
    values = sorted(values)
    return {key: values[min(len(values)-1, math.ceil(len(values)*fraction)-1)]
            for key, fraction in (('p50', .5), ('p95', .95), ('p99', .99), ('max', 1.))} if values else {}


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=90)
    if result.returncode:
        raise RuntimeError(json.dumps(dict(command=command, returncode=result.returncode,
                                          stdout=result.stdout, stderr=result.stderr)))
    return result.stdout


def parse(raw):
    generated, latency, reserve, rejection = [], [], [], []
    summary = None
    keys = ('input_targets', 'accepted', 'rejected', 'merged_inputs', 'rms_reference_target_error_rad',
            'max_reference_target_error_rad', 'reverse_velocity_response_ms', 'first_motion_clock_ms',
            'pause_to_rest_reference_ms', 'max_velocity', 'max_acceleration', 'max_jerk',
            'max_switch_1ms_delta_q', 'max_switch_1ms_delta_v', 'max_switch_1ms_delta_a',
            'excluded_expired_samples', 'first_nonzero_input_to_motion_ms', 'constant_speed_near_zero_samples',
            'constant_speed_restart_count', 'constant_speed_min_v', 'constant_speed_max_v', 'constant_speed_std_v',
            'axis0_rms_error_rad', 'axis7_rms_error_rad', 'waist_axis14_rms_error_rad')
    for line in raw.splitlines():
        kind, *items = line.split()
        if kind == 'GENERATE': generated.append(float(items[0]))
        elif kind == 'ACCEPT': latency.append(float(items[1])); reserve.append(float(items[2]))
        elif kind == 'REJECT': rejection.append(' '.join(items))
        elif kind == 'SUMMARY': summary = dict(zip(keys, map(float, items)))
    if summary is None:
        raise RuntimeError('PROBE_SUMMARY_MISSING')
    summary.update(generation_ms=statistics(generated), scheduled_source_to_reference_ms=statistics(latency),
                   local_commit_margin_ms=statistics(reserve), rejection_reasons=rejection)
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--vendor-root', type=Path, required=True)
    p.add_argument('--vendor-source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--baseline-ref', default='ed7b78c')
    p.add_argument('--inject-delay-ms', default='0,12,25', help='Non-RT generation delay added to actual measured compute')
    a = p.parse_args()
    if a.output.exists(): p.error('output must be fresh')
    delays = [int(v) for v in a.inject_delay_ms.split(',')]
    if any(not 0 <= v <= 100 for v in delays): p.error('delay outside 0..100 ms')
    a.output.mkdir(parents=True)
    native = ROOT/'src/control/bindu_execution/native'
    source = a.vendor_root/'tests/native/benchmark_position_latency.cpp'
    vendor = a.vendor_source/'servo_control/include'
    compiler = shutil.which('c++')
    paths = [Path(__file__).resolve(), source, vendor/'position_trajectory.h',
             *(vendor/'servo_control'/n for n in ('position_follower.h','position_watchdog.h','position_clock.h')),
             a.vendor_source/'robot_control/include/position_session_contract.h',
             *(native/n for n in ('interpolation.cpp', 'adaptive_stream.cpp', 'interpolation.hpp', 'adaptive_stream.hpp', 'curve_math.hpp'))]
    report = dict(schema=2, platform=platform.platform(), real_ros_dds=False, hardware_accessed=False,
        limits=dict(velocity=.25, acceleration=.5, jerk=162., original_target_ttl_ms=200,
                    old_pause_ttl_ms=200, target_pause_ttl_ms=2000),
        time_model=dict(sample_ms=1, old_ack_after_commit_ms=2, old_lead_ms=[60,30], new_local_budget_ms=10,
            actual_generation_time_rounded_up_to_ms=True, delayed_results_checked_by_production_commit=True,
            tracking_error_window_seconds=[0,4]),
        caveats=['Reference error uses desired target, not measured robot feedback.',
                 'Scheduled reference latency is a model, not ROS/DDS latency.',
                 'Switch 1ms deltas are not exact C2 evidence; exact C2 is tested separately.',
                 'Input merging count describes this bounded scheduling model, not DDS loss.',
                 'Ordinary inputs retain original source stamp/200ms TTL; target-stream PAUSE mints one separate fixed 2s source.',
                 'Old-path PAUSE remains 200ms; neither mode renews a source during waiting, merging or sampling.',
                 'Samples at or after original source deadline are excluded; native production tests cover watchdog behavior.',
                 'Constant-speed statistics cover 0.8..1.45s, within the 0.5..1.5s desired ramp.',
                 'Main RMS/max error covers all16 axes during the same 0..4s input-update window; reverse/first-motion/constant-speed metrics use axis0.',
                 'Separate axis0, axis7 and waist axis14 RMS errors cover the non-collinear interval.'],
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, results=[])
    try:
        with tempfile.TemporaryDirectory(prefix='target-compare-') as temp:
            temp = Path(temp)
            old = temp/'old'; old.mkdir()
            header = subprocess.check_output(['git', 'show', a.baseline_ref+':tools/deployment/snowbot_vendor/position_session/position_trajectory.h'], cwd=a.vendor_root)
            (old/'position_trajectory.h').write_bytes(header)
            report['baseline_ref'] = a.baseline_ref
            report['baseline_header_sha256'] = hashlib.sha256(header).hexdigest()
            binaries = {}
            for mode in ('old','target'):
                binary = temp/(mode+'-probe')
                command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread']
                if mode == 'target': command += ['-DTARGET_STREAM']
                command += ['-I'+str(old if mode == 'old' else vendor), '-I'+str(vendor),
                            '-I'+str(vendor/'bindu_execution_native'), str(source),
                            str(native/'interpolation.cpp'), str(native/'adaptive_stream.cpp'), '-o', str(binary)]
                run(command); binaries[mode] = binary
            for period, jitter in ((20,0),(10,0),(20,1),(10,1)):
                for delay in delays:
                    for mode, budget in (('old',60),('old',30),('target',10)):
                        label = f'{mode}-budget{budget}-period{period}-jitter{jitter}-delay{delay}'
                        raw = run([str(binaries[mode]), str(period), str(jitter), str(budget), str(delay), '2' if mode == 'old' else '0'])
                        (a.output/(label+'.log')).write_text(raw)
                        row = dict(mode=mode, local_budget_or_old_lead_ms=budget, target_period_ms=period,
                                   jitter=bool(jitter), injected_generation_delay_ms=delay, **parse(raw))
                        report['results'].append(row)
                        print(json.dumps({k:row[k] for k in ('mode','local_budget_or_old_lead_ms','target_period_ms','jitter','injected_generation_delay_ms','accepted','rejected','merged_inputs')}), flush=True)
        report['completed'] = True
        report['hashes_unchanged_after_run'] = all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest for p,digest in report['source_sha256'].items())
        if not report['hashes_unchanged_after_run']: raise RuntimeError('SOURCE_CHANGED_DURING_COMPARISON')
    except Exception as exc:
        report['completed'] = False; report['failure'] = str(exc)
        raise
    finally:
        (a.output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__': main()
