#!/usr/bin/env python3
"""Offline comparison of historical C++, scalar adaptive, and shared native strategies.

Explicit --legacy-root points to read-only historical sources. Build and CSV/JSON
outputs stay under --output. Identical one-axis targets and v/a/jerk limits; this
compares implementations, not language-independent algorithm speed or hardware.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(p.parent) for p in (ROOT/'src').rglob('package.xml')]
from bindu_contracts.profile import Profile
from bindu_execution.interpolation import State, fit_target, fit_online, fit_stop

HARNESS = r'''
#include "dexbot/interpolation/adaptive_interpolator.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
using namespace dexbot::interpolation;
using Clock = std::chrono::steady_clock;
int main() {
  AdaptiveInterpolator p;
  p.configure({.01,1.5,20.,400.}); p.reset({});
  double now, target; int update, stop;
  std::cout << std::setprecision(17);
  while(std::cin >> now >> target >> update >> stop) {
    auto stamp=Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(now)));
    auto begin=Clock::now();
    InterpolationSample s; p.step(stamp,&s);
    if(update) p.updateTarget({stamp,target,0.,0.});
    if(stop) p.clearTarget();
    double us=std::chrono::duration<double,std::micro>(Clock::now()-begin).count();
    std::cout << now << ',' << target << ',' << s.state.position_rad << ',' << s.state.velocity_rad_s << ','
              << s.state.acceleration_rad_s2 << ',' << s.state.jerk_rad_s3 << ',' << us << '\n';
  }
}
'''

EXPERIMENTAL_HARNESS = r'''
#include "adaptive_stream.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
using namespace bindu::execution;
int main() {
  AdaptiveStream p; p.reset(0.);
  double now,target; int update,stop;
  std::cout << std::setprecision(17);
  while(std::cin >> now >> target >> update >> stop) {
    auto begin=std::chrono::steady_clock::now();
    if(!p.step(now)) {std::cerr << "step failed at " << now << " q=" << p.state().q << '\n';return 2;}
    auto s=p.state();
    if(update&&!p.target(now,target,now+10.)) {std::cerr << "target failed at " << now << '\n';return 3;}
    if(stop) p.stop();
    double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
    std::cout << now << ',' << target << ',' << s.q << ',' << s.v << ',' << s.a << ',' << s.j << ',' << us << '\n';
  }
  std::cerr << p.guarded_steps();
}
'''


def inputs(case):
    now = 0.
    for i in range(600):
        dt = (.006,.014,.009,.011)[i%4] if case == 'jitter' else .01
        now += dt
        target = .6*math.sin(3*now) if case in ('sine','jitter') else .8
        if case == 'boundary_hold': target = 1.
        if case == 'near_boundary': target = .97+.029*math.sin(4*now)
        if case == 'micro_steps': target = .01 if int(now/.7)%2 else -.01
        if case == 'boundary_reversal': target = 1. if int(now/.35)%2 == 0 else -1.
        update = i == 0 or case in ('sine','jitter','boundary_reversal','near_boundary','micro_steps')
        stop = case == 'dropout' and i == 23
        yield now,target,int(update),int(stop)


def metrics(rows):
    # Backward differences are a sampled diagnostic, not the exact continuous
    # derivative. Their deviations include discretization at 10 ms.
    residuals=[0.,0.,0.]
    for left,right in zip(rows,rows[1:]):
        dt=right[0]-left[0]
        for k in range(3):
            residuals[k]=max(residuals[k],abs((right[k+2]-left[k+2])/dt-right[k+3]))
    us=sorted(r[-1] for r in rows)
    positions = [r[2] for r in rows]
    times = [r[0] for r in rows]
    derived_peaks = []
    for order in range(1,4):
        positions = [(positions[i+1]-positions[i])/(times[i+order]-times[i]) for i in range(len(positions)-1)]
        derived_peaks.append(max(abs(x)*math.factorial(order) for x in positions))
    tail=[r for r in rows if r[0]>=rows[-1][0]-1.]
    last_moving=max((i for i,r in enumerate(rows) if abs(r[3])>1e-6 or abs(r[4])>1e-6),default=-1)
    return dict(last_second_position_span=max(r[2] for r in tail)-min(r[2] for r in tail),
        last_second_peak_velocity=max(abs(r[3]) for r in tail),
        rest_since=rows[last_moving+1][0] if last_moving<len(rows)-1 else None,
        position_divided_difference_peaks=derived_peaks, rms_target_error=math.sqrt(statistics.mean((r[2]-r[1])**2 for r in rows)),
        peak_q=max(abs(r[2]) for r in rows),peak_v=max(abs(r[3]) for r in rows),
        peak_a=max(abs(r[4]) for r in rows),peak_j=max(abs(r[5]) for r in rows),
        sampled_derivative_discrepancy=residuals,compute_us_p50=statistics.median(us),
        compute_us_p99=us[int(.99*(len(us)-1))],compute_us_max=max(us),
        final_q=rows[-1][2],final_v=rows[-1][3])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which('c++')
    if not compiler: raise RuntimeError('C++ compiler required')
    harness=args.output/'legacy_harness.cpp';harness.write_text(HARNESS)
    binary=args.output/'legacy_harness'
    subprocess.run([compiler,'-std=c++17','-O2','-I'+str(args.legacy_root/'include'),
        str(args.legacy_root/'src/dexbot/interpolation/adaptive_interpolator.cpp'),str(harness),'-o',str(binary)],check=True)
    experimental_root=ROOT/'src/control/bindu_execution/native'
    experimental_harness=args.output/'experimental_harness.cpp'
    experimental_harness.write_text(EXPERIMENTAL_HARNESS)
    experimental_binary=args.output/'experimental_harness'
    subprocess.run([compiler,'-std=c++17','-O2','-Wall','-Wextra','-I'+str(experimental_root),
                   str(experimental_root/'adaptive_stream.cpp'),str(experimental_harness),'-o',str(experimental_binary)],check=True)
    profile=Profile('comparison',{'arm':['q']},{'q':[-1.,1.]},1.5,('joint_position',),'comparison')
    results={}
    for case in ('step','boundary_hold','sine','boundary_reversal','dropout','jitter','near_boundary','micro_steps'):
        samples=list(inputs(case))
        wire=''.join(f'{t} {q} {update} {stop}\n' for t,q,update,stop in samples)
        legacy=subprocess.run([str(binary)],input=wire,text=True,capture_output=True,check=True)
        rows=[list(map(float,line.split(','))) for line in legacy.stdout.splitlines()]
        experiment=subprocess.run([str(experimental_binary)],input=wire,text=True,capture_output=True)
        if experiment.returncode: raise RuntimeError(case+': '+experiment.stderr)
        experimental_rows=[list(map(float,line.split(','))) for line in experiment.stdout.splitlines()]
        results[case]={'legacy':metrics(rows),'scalar_adaptive':metrics(experimental_rows),
                       'scalar_guarded_steps':int(experiment.stderr)}
        datasets=[('legacy',rows),('scalar_adaptive',experimental_rows)]
        for label,fitter in (('native_quintic',fit_target),('native_online',fit_online)):
            new=[];path=None;state=State.rest((0.,));last_goal=None;rejected=0
            for t,q,update,stop in samples:
                start=time.perf_counter()
                state=path.sample(t) if path else state
                if stop:
                    path=fit_stop(profile,('q',),state,t)
                elif update and q != last_goal:
                    try:
                        path=fitter(profile,('q',),state,(q,),t)
                        last_goal=q
                    except ValueError:
                        rejected+=1
                elapsed=(time.perf_counter()-start)*1e6
                new.append([t,q,state.q[0],state.v[0],state.a[0],state.j[0],elapsed])
            results[case][label]=metrics(new)
            results[case][label+'_rejected_updates']=rejected
            datasets.append((label,new))
        for label,data in datasets:
            with (args.output/f'{case}_{label}.csv').open('w') as f:
                writer=csv.writer(f);writer.writerow(['time','target','q','v','a','j','compute_us']);writer.writerows(data)
    names=tuple('q'+str(i) for i in range(7))
    seven=Profile('seven',{'arm':list(names)},{n:[-1.,1.] for n in names},1.5,('joint_position',),'seven')
    path=None;state=State.rest((0.,)*7);timings=[]
    for i in range(1000):
        t=i*.01;target=tuple(.4*math.sin(2*t+k*.03) for k in range(7))
        begin=time.perf_counter()
        state=path.sample(t) if path else state
        path=fit_online(seven,names,state,target,t)
        timings.append((time.perf_counter()-begin)*1e6)
    seven_metrics={'updates':1000,'p50_us':statistics.median(timings),
                   'p99_us':sorted(timings)[989],'max_us':max(timings)}
    sources=[args.legacy_root/'src/dexbot/interpolation/adaptive_interpolator.cpp',
             ROOT/'src/control/bindu_execution/bindu_execution/interpolation.py',
             experimental_root/'adaptive_stream.cpp',experimental_root/'adaptive_stream.hpp',
             experimental_root/'interpolation.cpp',experimental_root/'interpolation.hpp',
             experimental_root/'curve_math.hpp',experimental_root/'python_bindings.cpp',Path(__file__).resolve()]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    result={'platform':platform.platform(),'python':sys.version,'limits':{'q':[-1,1],'v':1.5,'a':20,'j':400},
            'source_sha256':hashes,'seven_axis_native_online':seven_metrics,'control_period':.01,'cases':results,'notes':[
                'Historical kernel advances a fixed 10 ms even in the jitter case.',
                'All implementations publish the current state then apply the newly received target for subsequent motion.',
                'Scalar C++ variants use identical compiler flags; native strategies include the Python binding overhead.',
                'Runtime online targets use native_online; native_quintic provides synchronized P2P, timed curves and stopping.',
                'Dropout uses legacy clearTarget versus a certified stop; RMS target error is not a dropout success metric.',
                'Sampled derivative discrepancy contains finite-difference error and does not alone establish a constraint violation.']}
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
