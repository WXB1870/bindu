"""Orin/Humble navigation-only preparation; never starts a chassis driver."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys

from tools import ROOT
from tools.deployment.x86_real import clean_environment, OVERLAYS, SYSTEM_PATH

# Allow environment/config checks from a fresh clone, before the ROS build exists.
sys.path.insert(0, str(ROOT/'src/capabilities/bindu_navigation'))
from bindu_navigation.orin import load_site, site_errors, MOTION_BLOCK

TEMPLATE = ROOT/'src/integration/bindu_runtime/config/navigation/navigation_orin.json'
PACKAGES = ('bindu_contracts','bindu_interfaces','bindu_execution','bindu_hardware',
            'bindu_recording','bindu_navigation','bindu_runtime')
BASE_APT = ('build-essential','cmake','git','python3-dev','python3-colcon-common-extensions',
            'python3-rosdep','python3-numpy','python3-yaml','ros-humble-ros-base','ros-humble-nav2-msgs',
            'ros-humble-nav2-map-server','ros-humble-nav2-controller',
            'ros-humble-nav2-planner','ros-humble-nav2-bt-navigator','ros-humble-nav2-lifecycle-manager',
            'ros-humble-nav2-navfn-planner','ros-humble-nav2-regulated-pure-pursuit-controller',
            'ros-humble-rviz2','ros-humble-tf2-ros','ros-humble-tf2-msgs','ros-humble-sensor-msgs-py')
ROUTE_APT = {'2d': ('ros-humble-slam-toolbox','ros-humble-nav2-amcl'),
             'lio': ('libpcl-dev','libeigen3-dev','python3-scipy','python3-open3d','ros-humble-pcl-conversions')}


def target_errors(info, machine, python):
    errors = []
    if info.get('ID') != 'ubuntu' or info.get('VERSION_ID') != '22.04': errors.append('需要 Ubuntu 22.04')
    if machine not in ('aarch64','arm64'): errors.append('需要 ARM64；x86只能作替身验证，禁止复制二进制到Orin')
    if tuple(python[:2]) != (3,10): errors.append('需要系统 Python 3.10，与Humble ABI一致')
    return errors


def environment():
    data = {'os':platform.freedesktop_os_release(),'architecture':platform.machine(),
            'python':list(sys.version_info[:2]),'memory_gb':None,'model':None,'l4t':None,'jetpack':None}
    for key, file in [('model','/proc/device-tree/model'),('l4t','/etc/nv_tegra_release')]:
        path = Path(file)
        if path.is_file(): data[key] = path.read_text().strip('\0\n')
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemTotal:'): data['memory_gb'] = round(int(line.split()[1])/1024**2,2)
    result = subprocess.run(['dpkg-query','-W','-f=${Version}','nvidia-jetpack'],text=True,capture_output=True)
    if result.returncode == 0: data['jetpack'] = result.stdout
    data['errors'] = target_errors(data['os'],data['architecture'],data['python'])
    return data


def digest():
    h = hashlib.sha256()
    tracked = subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z',
                                       '--','src','tools/deployment','tools/run_navigation.sh'],cwd=ROOT)
    names = set(PACKAGES+('bindu_lio',))
    for raw in sorted(set(path for path in tracked.split(b'\0') if path)):
        relative = Path(os.fsdecode(raw))
        selected = bool(set(relative.parts) & names) or str(relative) in (
            'tools/deployment/orin_navigation.py','tools/deployment/x86_real.py','tools/run_navigation.sh')
        if selected and relative.suffix not in ('.md','.pyc'):
            h.update(raw+b'\0'); h.update((ROOT/relative).read_bytes())
    return h.hexdigest()


def env_script(workspace, route):
    reader = ('import json,sys; p=json.load(open(sys.argv[1])).get("driver_setup"); '
              'assert p is None or (isinstance(p,str) and p.startswith("/") and '
              'p.endswith("/local_setup.bash") and chr(10) not in p); print(p or "")')
    return '\n'.join([
        '# Generated locally; source from bash. Do not copy install/build from another machine.',
        'unset '+' '.join(OVERLAYS), 'export PATH='+shlex.quote(SYSTEM_PATH),
        'export BINDU_NAVIGATION_ONLY=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1',
        'export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=1',
        'source /opt/ros/humble/setup.bash || return',
        'if [[ -f '+shlex.quote(str(workspace/'install/local_setup.bash'))+' ]]; then',
        '  source '+shlex.quote(str(workspace/'install/local_setup.bash'))+' || return', 'fi',
        # Read a configured path as data; never eval a JSON-derived shell command.
        'bindu_nav_driver_setup="$(/usr/bin/python3 -c '+shlex.quote(reader)+' '+shlex.quote(str(workspace/'site.json'))+')" || return',
        'if [[ -n "$bindu_nav_driver_setup" ]]; then source "$bindu_nav_driver_setup" || return; fi',
        'unset bindu_nav_driver_setup', ''])


def build_command(workspace, route):
    return ['/usr/bin/python3','/usr/bin/colcon','--log-base',str(workspace/'log'),'build',
            '--base-paths',str(ROOT/'src'),'--packages-up-to','bindu_runtime',
            *(['bindu_lio'] if route == 'lio' else []),
            '--build-base',str(workspace/'build'),'--install-base',str(workspace/'install'),
            '--executor','sequential','--cmake-args','-DCMAKE_BUILD_TYPE=Release',
            '-DPython3_EXECUTABLE=/usr/bin/python3','-DPYTHON_EXECUTABLE=/usr/bin/python3']


def run_env(workspace, command):
    return subprocess.run(['bash','--noprofile','--norc','-ec','source "$1"; shift; exec "$@"',
                           'bindu-orin',str(workspace/'env.sh'),*map(str,command)],
                          cwd=ROOT,env=clean_environment(),check=True)


def deployment(workspace):
    data = json.loads((workspace/'deployment.json').read_text())
    host = environment()
    if host['errors']: raise ValueError('; '.join(host['errors']))
    if data['root'] != str(ROOT) or data['workspace'] != str(workspace):
        raise ValueError('WORKSPACE_MOVED: 在目标机新目录执行prepare，不复用x86/install/venv/.so')
    site = load_site(workspace/'site.json')
    if site['route'] != data['route']: raise ValueError('ROUTE_CHANGED: 换路线请用新的--workspace重新prepare')
    return site


def require_build(workspace):
    data = json.loads((workspace/'build.json').read_text())
    if not data['ready'] or data['digest'] != digest(): raise ValueError('BUILD_REQUIRED: 源码已变更或构建未成功，请重新build')


def rviz_text(site):
    # JSON is valid YAML; generate from the same site so topics cannot drift.
    ns = site['namespace']
    return json.dumps({'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],
        'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':site['frames']['map'],'Frame Rate':15},
            'Displays':[
                {'Class':'rviz_default_plugins/Grid','Name':'Grid','Enabled':True},
                {'Class':'rviz_default_plugins/TF','Name':'TF','Enabled':True},
                {'Class':'rviz_default_plugins/Map','Name':'Saved or live map','Enabled':True,
                 'Topic':{'Value':ns+'/navigation/map','Durability Policy':'Transient Local','Reliability Policy':'Reliable'}},
                {'Class':'rviz_default_plugins/LaserScan','Name':'Measured scan','Enabled':True,
                 'Topic':{'Value':site['scan_topic'],'Reliability Policy':'Best Effort'},'Size (m)':.03},
                {'Class':'rviz_default_plugins/Odometry','Name':'Measured odometry','Enabled':True,
                 'Topic':{'Value':site['odom_topic'],'Reliability Policy':'Best Effort'}}],
            # No Nav2 goal tool: it bypasses the public execution action.
            'Tools':[{'Class':'rviz_default_plugins/Interact'}, {'Class':'rviz_default_plugins/MoveCamera'},
                     {'Class':'rviz_default_plugins/SetInitialPose','Topic':ns+'/initialpose'}]},
        'Window Geometry':{'Width':1200,'Height':800}},ensure_ascii=False,indent=2)+'\n'


def save_map(workspace, site):
    from bindu_runtime.navigation_preflight import run
    errors = run(site,'mapping')
    if errors: raise ValueError('\n'.join(errors))
    prefix = Path(site['map']['prefix'])
    for suffix in ('.yaml','.pgm','.png','.posegraph','.data'):
        if Path(str(prefix)+suffix).exists(): raise ValueError('MAP_ALREADY_EXISTS: 换map.prefix；不覆盖已有批次')
    if site['route'] == 'lio' and Path(str(prefix)+'.pcd').exists():
        raise ValueError('PCD_ALREADY_EXISTS: 检查是否已部分保存；保留该批次，用新批次重新建图')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    if site['route'] == 'lio':
        from std_srvs.srv import Trigger
        call(site['namespace']+'/lio/save_map',Trigger,Trigger.Request())
        if not Path(str(prefix)+'.pcd').is_file(): raise ValueError('PCD_NOT_SAVED: 查看fast_lio日志')
    else:
        from slam_toolbox.srv import SerializePoseGraph
        call(site['namespace']+'/slam_toolbox/serialize_map',SerializePoseGraph,
             SerializePoseGraph.Request(filename=str(prefix)))
        if not all(Path(str(prefix)+s).is_file() for s in ('.posegraph','.data')):
            raise ValueError('POSE_GRAPH_NOT_SAVED: 查看slam_toolbox日志，保留部分文件')
    subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix),
                    '--ros-args','-p','map_subscribe_transient_local:=true',
                    '-r','map:='+site['namespace']+'/navigation/map'],check=True,timeout=30)
    from bindu_navigation.orin import map_errors
    saved = dict(site, map=dict(site['map'],yaml=str(prefix)+'.yaml',pcd=str(prefix)+'.pcd'))
    errors = map_errors(saved)
    if errors: raise ValueError('\n'.join(errors))
    print('已保存并检查地图文件。将 site.json 的 map.yaml 设为 '+str(prefix)+'.yaml；LIO还需设置map.pcd。')


def call(name, kind, request):
    import rclpy
    rclpy.init(); node = rclpy.create_node('bindu_map_save_client')
    try:
        client = node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=5): raise ValueError('SERVICE_UNAVAILABLE: '+name)
        future = client.call_async(request); rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.done(): raise ValueError('SERVICE_TIMEOUT: '+name+'；检查日志/部分地图文件，不自动重试覆盖')
        reply = future.result()
        if hasattr(reply,'success') and not reply.success: raise ValueError(str(reply))
        if hasattr(reply,'result') and reply.result != 0: raise ValueError(str(reply))
    finally:
        node.destroy_node(); rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('environment','dependencies','prepare','build','check','start','save-map','rviz','manual','navigate','cancel','stop'))
    parser.add_argument('--workspace',type=Path,default=ROOT/'artifacts/orin-navigation')
    parser.add_argument('--site',type=Path,help='check可在未prepare时检查任意现场配置；其他命令使用workspace/site.json')
    parser.add_argument('--route',choices=('2d','lio'))
    parser.add_argument('--stage',choices=('config','sensors','mapping','localization','navigation'),default='config')
    parser.add_argument('--phase',choices=('mapping','localization','navigation'),default='mapping')
    args = parser.parse_args(argv); workspace = args.workspace.expanduser().resolve()
    try:
        if args.command in ('manual','navigate','cancel','stop'):
            raise ValueError(MOTION_BLOCK+' 此入口未装配可控的真机运动会话，无法确认停车；现场请使用底盘已验证的人工急停。')
        if args.command == 'environment':
            data = environment(); print(json.dumps(data,indent=2,ensure_ascii=False)); return 2 if data['errors'] else 0
        if args.command == 'dependencies':
            if not args.route: raise ValueError('请显式选择 --route 2d 或 lio')
            print('sudo apt install --no-install-recommends '+' '.join(BASE_APT+ROUTE_APT[args.route]))
            print('# 先配置官方Humble软件源；传感器驱动依型号另装。此命令只输出依赖，不执行安装。')
            return 0
        if args.site and args.command != 'check': raise ValueError('--site 仅用于只读check；请编辑workspace/site.json')
        if args.command == 'check':
            site = load_site(args.site or workspace/'site.json')
            errors = site_errors(site, 'sensors' if args.stage=='config' else args.stage)
            if args.stage != 'config' and not errors:
                from bindu_runtime.navigation_preflight import run
                errors += run(site,args.stage)
            if errors: raise ValueError('\n'.join(errors))
            print('PASS: '+args.stage+'；仅此阶段预检通过，不代表允许运动或真机验收'); return 0
        if args.command == 'prepare':
            data = environment()
            if data['errors']: raise ValueError('; '.join(data['errors']))
            if not args.route: raise ValueError('请先确认传感器条件，再显式指定 --route')
            if not Path('/opt/ros/humble/setup.bash').is_file(): raise ValueError('ROS_NOT_INSTALLED: 先安装官方Humble基础环境')
            if workspace.exists() and any(workspace.iterdir()): raise ValueError('WORKSPACE_NOT_EMPTY: 保留旧目录，使用新部署目录')
            workspace.mkdir(parents=True,exist_ok=True)
            site = load_site(TEMPLATE); site['route'] = args.route
            (workspace/'site.json').write_text(json.dumps(site,indent=2,ensure_ascii=False)+'\n')
            (workspace/'deployment.json').write_text(json.dumps(dict(root=str(ROOT),workspace=str(workspace),route=args.route,environment=data),indent=2)+'\n')
            (workspace/'env.sh').write_text(env_script(workspace,args.route))
            print('已准备 '+str(workspace)+'；填写site.json后构建。未安装依赖、启动设备或修改网络。'); return 0
        site = deployment(workspace)
        if args.command == 'build':
            before = digest(); info = dict(ready=False,digest=before)
            (workspace/'build.json').write_text(json.dumps(info))
            run_env(workspace,build_command(workspace,site['route']))
            if before != digest(): raise ValueError('SOURCE_CHANGED_DURING_BUILD: 请重新构建')
            info['ready'] = True; (workspace/'build.json').write_text(json.dumps(info,indent=2)+'\n')
            return 0
        require_build(workspace)
        errors = site_errors(site,args.phase if args.command=='start' else 'sensors')
        if errors: raise ValueError('\n'.join(errors))
        if args.command == 'start':
            from bindu_runtime.navigation_preflight import run
            errors = run(site,'sensors')
            if errors: raise ValueError('\n'.join(errors))
            run_env(workspace,['ros2','launch','bindu_runtime','navigation_orin.launch.py',
                               'site:='+str(workspace/'site.json'),'phase:='+args.phase])
        elif args.command == 'save-map': save_map(workspace,site)
        elif args.command == 'rviz':
            path = workspace/'navigation.rviz'; path.write_text(rviz_text(site))
            run_env(workspace,['rviz2','-d',str(path),'--ros-args','-r','/tf:='+site['tf_topic'],
                               '-r','/tf_static:='+site['tf_static_topic']])
        return 0
    except (OSError,ValueError,KeyError,TypeError,ImportError,subprocess.SubprocessError) as exc:
        print('Orin导航：'+str(exc),file=sys.stderr); return 2


if __name__ == '__main__': raise SystemExit(main())
