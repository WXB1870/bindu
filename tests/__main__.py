"""Select existing test suites without starting a runtime on import."""
import argparse
import subprocess
import sys

from tests.support.paths import ROOT

PICO_TESTS=('test_pico_control','test_pico_loading','test_pico_mapping','test_pico_hud','test_pico_grasp')
SUITES={
    'core':('test_interpolation','test_execution_fault_sequences','test_system_contracts','test_runtime_mode'),
    'pico':PICO_TESTS,
    'vr':PICO_TESTS+('test_independent_arms','test_vr_timing','test_one_euro','test_teleoperation','test_teleop_modes',
          'test_snowbot_adapter','test_snowbot_gripper','test_snowbot_physics','test_snowbot_isaac_io',
          'test_snowbot_position_session','test_snowbot_live','test_snowbot_errors','test_snowbot_diagnostics','test_snowbot_pdo','test_teleop_deployment'),
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list',action='store_true',help='List groups and modules; do not import or run tests')
    parser.add_argument('group',nargs='?',choices=('unit','integration','physics','replay'))
    parser.add_argument('target',nargs='?',help='unit: vr/pico/core/all/test_name; other groups: scenario name')
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if args.list:
        for group in ('unit','integration','physics','replay','support','native'):
            print(group+': '+', '.join(p.stem for p in sorted((ROOT/'tests'/group).glob('*'))
                  if p.suffix in ('.py','.cpp') and p.stem!='__init__'))
        print('unit suites: '+', '.join((*SUITES,'all')))
        return 0
    if not args.group:
        parser.print_help();return 0
    if not args.target:parser.error('Select an explicit target; inspect --list first')
    extra=args.arguments[1:] if args.arguments[:1]==['--'] else args.arguments
    if args.group=='unit':
        if args.target=='all':
            command=[sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests/unit'),
                     '-t',str(ROOT),'-p','test_*.py','-v',*extra]
        else:
            names=SUITES.get(args.target,(args.target,))
            if any(not (ROOT/'tests/unit'/(name+'.py')).is_file() for name in names):parser.error('Unknown unit suite/module')
            command=[sys.executable,'-m','unittest','-v',*['tests.unit.'+name for name in names],*extra]
    else:
        aliases={'position':'validate_position_teleop','axes':'validate_vr_axes','vr':'replay_vr','shovel':'validate_shovel_isaac'}
        name=aliases.get(args.target,args.target)
        if not name.startswith(('validate_','replay_')):name='validate_'+name
        if not (ROOT/'tests'/args.group/(name+'.py')).is_file():parser.error('Unknown scenario for this group')
        if args.group=='physics' and name=='validate_shovel_isaac':
            command=['bash',str(ROOT/'tools/run_shovel_physics.sh'),'--kinematic',*extra]
        else:command=[sys.executable,'-m','tests.'+args.group+'.'+name,*extra]
    print('Running: '+' '.join(command),flush=True)
    return subprocess.call(command,cwd=ROOT)


if __name__=='__main__':
    raise SystemExit(main())
