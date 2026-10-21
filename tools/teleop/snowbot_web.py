"""Loopback-only Meshcat operator view; all robot feedback is read-only."""
import json
import asyncio
import threading
from pathlib import Path
from tools import ROOT
import queue
import time


def meshcat_urdf(source, output):
    """Keep the CAD link tree; omit explicitly empty STL placeholders in the view."""
    import xml.etree.ElementTree as ET
    source = Path(source)
    tree = ET.parse(source)
    package, skipped = source.parent.parent, []
    for link in tree.findall('link'):
        for element in list(link.findall('visual'))+list(link.findall('collision')):
            mesh = element.find('geometry/mesh')
            if mesh is None:
                continue
            uri = mesh.get('filename')
            prefix = f'package://{package.name}/'
            path = package/uri.removeprefix(prefix) if uri.startswith(prefix) else Path(uri)
            if not path.is_absolute():
                raise ValueError('UNRESOLVED_MESHCAT_MESH:'+uri)
            raw = path.read_bytes()
            if path.suffix.lower()=='.stl' and len(raw)==84 and raw[80:]==b'\x00'*4:
                skipped.append(dict(link=link.get('name'),kind=element.tag,mesh=str(path)))
                link.remove(element)
            else:
                mesh.set('filename',str(path))
    target = Path(output)/'viewer.urdf'
    tree.write(target,encoding='utf-8',xml_declaration=True)
    (Path(output)/'model-import.json').write_text(json.dumps(dict(source=str(source),
        skipped_empty_meshes=skipped),ensure_ascii=False,indent=2))
    return target


def project_state(data, now, identity):
    """Do not let a recently read file make stale robot/input data look live."""
    from tools.teleop.snowbot_live import validate_state
    result = {k:data.get(k) for k in ('run_id','written_at','feedback_stamp','execution','code')}
    result.update(valid=False, pico_fresh=False, pico={}, server_time=now)
    result['diagnostics']=data.get('diagnostics',{})
    if identity.get('runtime_mode') == 'real' and identity.get('device_backend') == 'snowbot_vendor':
        pdo = json.loads(json.dumps(data.get('pdo', {})))
        stamp = pdo.get('checked_at')
        if type(stamp) not in (float, int) or not 0 <= now-stamp <= .2:
            pdo.update(pdo_valid=False, reasons=[*pdo.get('reasons', []), 'PDO_VIEW_STALE'])
        result['pdo'] = pdo
    physics=data.get('physics',{})
    if (identity.get('runtime_mode')=='simulation' and identity.get('device_backend')=='external_simulation' and
            physics.get('run_id')==identity['run_id'] and physics.get('profile_hash')==identity['profile_hash'] and
            0 <= now-physics.get('stamp',0) <= .2):
        result['physics']=physics
    try:
        validate_state(data, now, identity['run_id'], identity['profile_hash'], identity['joint_names'],
                       identity.get('runtime_mode','simulation'), identity.get('device_backend','kinematic'))
        if identity.get('instance_id') and data.get('instance_id') != identity['instance_id']:
            raise ValueError('LIVE_EXECUTOR_CHANGED')
        if not identity.get('instance_id'):
            identity['instance_id'] = data.get('instance_id')
        result['valid'] = True
        pico = data.get('pico', {})
        if (pico.get('run_id') == identity['run_id'] and
                pico.get('profile_hash') == identity['profile_hash'] and
                pico.get('runtime_mode') == identity.get('runtime_mode','simulation') and
                0 <= now-pico.get('stamp', 0.) <= .6):
            pico = json.loads(json.dumps(pico))
            elapsed = now-pico['stamp']
            for field in ('input_age_s','raw_input_age_s','feedback_age_s'):
                if type(pico.get(field)) in (float,int):
                    pico[field] += elapsed
            pico['input_fresh'] = bool(pico.get('input_fresh') and
                type(pico.get('input_age_s')) in (float,int) and 0 <= pico['input_age_s'] <= .25)
            if not pico['input_fresh']:
                for hand in pico.get('controller_states', {}).values():
                    hand.update(valid=False,squeeze=None,trigger=None,buttons=None,
                                button_edges=None,position_m=None)
            result.update(pico=pico, pico_fresh=True)
            errors = data.get('end_effector_errors', {})
            if (errors and type(errors.get('stamp')) in (float,int) and
                    0 <= data['feedback_stamp']-errors['stamp'] <= .12 and 0 <= now-errors['stamp'] <= .2):
                result['end_effector_errors'] = errors
    except (ValueError, TypeError) as exc:
        result['code'] = str(exc)
    return result


def tool_display_pose(data, left_pose, mount, now):
    """Use measured tool feedback when available, otherwise show a FK preview.

    The preview uses the same geometry/mount as collision checking. It does not
    declare a tool loaded or change the independent/coupled control mode.
    The caller must first validate joint feedback and session identity.
    """
    if data.get('pico', {}).get('teleop_mode') != 'bimanual_tool':
        return None, 'inactive'
    if data['device_backend'] == 'external_simulation':
        physics = data.get('physics', {})
        if (physics.get('run_id') != data['run_id'] or
                physics.get('profile_hash') != data['profile_hash'] or
                not 0 <= now-physics.get('stamp', 0.) <= .2):
            return None, 'unavailable'
        if physics.get('tool_present'):
            from tools.teleop.snowbot_errors import transform
            return transform(physics['tool_pose']), 'measured'
    return left_pose @ mount, 'preview'


class EventLog:
    """Small local diagnostic stream, independent of control and training recording."""
    def __init__(self, path=None):
        self.path, self.events, self.previous, self.sequence = Path(path) if path else None, [], None, 0

    def sample(self, state):
        p = state.get('pico', {})
        hands = p.get('controller_states', {})
        connection = (state['valid'], state['pico_fresh'], p.get('raw_input_valid'), p.get('input_fresh'))
        key = (p.get('phase'), p.get('reason'), p.get('source_id'))
        prior = self.previous
        def add(kind, message, warning=False):
            self.sequence += 1
            event = dict(id=self.sequence, time=state['server_time'], kind=kind,
                         run_id=state.get('run_id'),
                         message=message, phase=p.get('phase'), reason=p.get('reason'),
                         level='warning' if warning else 'info')
            self.events.append(event)
            del self.events[:-300]
            if self.path:
                with self.path.open('a') as stream:
                    stream.write(json.dumps(event, ensure_ascii=False)+'\n')
        if prior is None or connection != prior[0]:
            add('connection', '反馈'+('有效' if state['valid'] else '失效')+' · VR '+
                ('追踪有效' if state['pico_fresh'] and p.get('raw_input_valid') and p.get('input_fresh') else '未就绪'),
                not state['valid'] or not p.get('input_fresh'))
        if prior is None or key != prior[1]:
            add('state', (p.get('phase') or 'unknown')+' · '+(p.get('reason') or state.get('code') or 'WAITING'),
                p.get('phase') == 'locked')
        if prior and state['pico_fresh'] and p.get('input_fresh'):
            for side, keys in (('left',('X','Y','摇杆','扳机')),('right',('A','B','摇杆','扳机'))):
                current, before = hands.get(side, {}), prior[2].get(side, {})
                if not current.get('valid') or not before.get('valid'):
                    continue
                edges, old = current.get('button_edges'), before.get('button_edges')
                if edges and old and key[2] == prior[1][2]:
                    for i, (a,b) in enumerate(zip(edges,old)):
                        if a > b:
                            add('input', ('左' if side=='left' else '右')+' '+keys[i//2]+
                                (' 按下' if i%2==0 else ' 松开')+(f' ×{a-b}' if a-b>1 else ''))
                a,b = current.get('squeeze'),before.get('squeeze')
                if a is not None and b is not None and (a>.7) != (b>.7):
                    add('input', ('左' if side=='left' else '右')+'侧握 '+('握紧' if a>.7 else '释放'))
        self.previous = (connection, key, hands)


def _server(read_state, trace_file, output, port, ready, views, identity, stop, model_status):
    asyncio.set_event_loop(asyncio.new_event_loop())
    import tornado.web
    import tornado.ioloop
    from meshcat.servers.zmqserver import (ZMQWebSocketBridge, WebSocketHandler,
        StaticFileHandlerNoCache, VIEWER_ROOT, VIEWER_HTML)
    html = ROOT/'tools/web/snowbot.html'
    log = EventLog(Path(output)/'events.jsonl' if trace_file else None)
    latest = {}
    client_status = {}
    http_servers = []

    def sample():
        nonlocal latest
        try:
            data = read_state()
            latest = project_state(data, time.time(), identity)
        except (OSError, ValueError, TypeError):
            latest = dict(valid=False, pico_fresh=False, server_time=time.time(), pico={}, code='STATE_UNAVAILABLE')
        log.sample(latest)
        latest.update(events=log.events[-40:], recording=False)
        if trace_file:
            latest['recording'] = bool(latest.get('diagnostics',{}).get('active'))
        model = model_status.read()
        if 0 <= time.time()-model.get('updated_at', 0.) <= 2.:
            latest['model'] = model
        if 0 <= time.time()-client_status.get('received_at', 0.) <= 8.:
            latest['browser'] = client_status
        return latest

    class PageSocket(WebSocketHandler):
        def get(self, *args, **kwargs):
            if self.request.headers.get('Upgrade','').lower() == 'websocket':
                return super().get(*args, **kwargs)
            self.set_header('Content-Type','text/html; charset=utf-8')
            self.set_header('Cache-Control','no-store')
            self.write(html.read_text())

        def check_origin(self, origin):
            from urllib.parse import urlsplit
            return urlsplit(origin).hostname in ('127.0.0.1','localhost')

    class State(tornado.web.RequestHandler):
        def get(self):
            self.set_header('Cache-Control','no-store')
            self.write(sample())

    class View(tornado.web.RequestHandler):
        def post(self):
            from urllib.parse import urlsplit
            origin = self.request.headers.get('Origin')
            if origin and urlsplit(origin).hostname not in ('127.0.0.1','localhost'):
                raise tornado.web.HTTPError(403)
            try:
                value = json.loads(self.request.body)
                if value.get('action') not in ('grid','tool'):
                    raise ValueError('unknown view')
                if value['action'] in ('grid','tool') and type(value.get('enabled')) is not bool:
                    raise ValueError('invalid toggle')
                views.put_nowait(value)
            except (ValueError, queue.Full):
                raise tornado.web.HTTPError(400)
            self.write({'ok':True})

    class Export(tornado.web.RequestHandler):
        def get(self):
            self.set_header('Content-Type','application/json; charset=utf-8')
            self.set_header('Content-Disposition','attachment; filename="bindu-diagnostics.json"')
            self.write(json.dumps(dict(snapshot=sample(),events=log.events),ensure_ascii=False,indent=2))

    class ClientReport(tornado.web.RequestHandler):
        def post(self):
            nonlocal client_status
            from urllib.parse import urlsplit
            if urlsplit(self.request.headers.get('Origin','')).hostname not in ('127.0.0.1','localhost'):
                raise tornado.web.HTTPError(403)
            if len(self.request.body)>4096:raise tornado.web.HTTPError(400)
            try:
                data=json.loads(self.request.body)
                allowed={'error','model_meshes','visible_meshes','camera','direction','target','bounds','canvas','socket'}
                if not isinstance(data,dict) or set(data)-allowed:raise ValueError()
                data['received_at']=time.time()
                json.dumps(data,allow_nan=False)
                client_status = data
            except (ValueError,TypeError):raise tornado.web.HTTPError(400)
            self.write({'ok':True})

    class LocalApplication(tornado.web.Application):
        def listen(self, port, **kwargs):
            # Meshcat 0.3.2 otherwise binds its HTTP socket on all interfaces.
            server = super().listen(port, address='127.0.0.1', **kwargs)
            http_servers.append(server)
            return server

    class LocalBridge(ZMQWebSocketBridge):
        def make_app(self):
            return LocalApplication([
                (r'/', PageSocket, {'bridge':self}),
                (r'/api/state', State), (r'/api/view', View), (r'/export', Export),
                (r'/api/client',ClientReport),
                (r'/static/(.*)', StaticFileHandlerNoCache,
                 {'path':VIEWER_ROOT,'default_filename':VIEWER_HTML})])

    bridge = None
    loop = tornado.ioloop.IOLoop.current()
    sample_errors = []
    def tick():
        if stop.is_set():
            loop.stop()
        else:
            try:
                sample()
            except Exception as exc:
                sample_errors.append(exc)
                stop.set()
                loop.stop()
    timer = tornado.ioloop.PeriodicCallback(tick,100)
    try:
        bridge = LocalBridge(host='127.0.0.1', port=port)
        sample()
        timer.start()
        ready.put((bridge.zmq_url, f'http://127.0.0.1:{port}/'))
        bridge.run()
        if sample_errors:
            raise sample_errors[0]
    except Exception as exc:
        try:
            ready.put_nowait(exc)
        except queue.Full:
            pass
        raise
    finally:
        timer.stop()
        for server in http_servers:
            server.stop()
        if bridge:
            for socket in list(bridge.websocket_pool):
                socket.close()
            bridge.zmq_stream.close(linger=0)
        loop.close(all_fds=True)


def serve(args, read_state=None, stop=None):
    import hashlib
    import numpy as np
    import pinocchio as pin
    import meshcat
    import meshcat.geometry as geom
    from pinocchio.visualize import MeshcatVisualizer
    from tools.teleop.snowbot_live import validate_state, display_tool_config, LatestState
    stop = stop or threading.Event()
    if read_state is None:
        read_state = lambda: json.loads(args.state_file.read_text())
    deadline = time.monotonic() + args.startup_timeout
    while not stop.is_set():
        data = read_state()
        try:
            # Metadata is available before hardware feedback. Render a waiting
            # page without turning absent feedback into a startup failure.
            if data.get('run_id') != args.run_id or not data.get('display_urdf'):
                raise ValueError('WAITING_FOR_METADATA')
            pico = data.get('pico', {})
            if (pico.get('run_id') != args.run_id or
                    not 0 <= time.time()-pico.get('stamp', 0.) <= .6):
                raise ValueError('WAITING_FOR_PICO')
            data.update(runtime_mode=args.runtime_mode,device_backend=args.device_backend)
            break
        except (ValueError, TypeError):
            if time.monotonic() >= deadline:
                raise TimeoutError('等待遥操作状态超时，请检查控制终端日志。')
            stop.wait(.05)
    else:
        return
    model_path = Path(data['display_urdf'])
    if hashlib.sha256(model_path.read_bytes()).hexdigest() != data['display_sha256']:
        raise ValueError('LIVE_MODEL_CHANGED')
    display_path = meshcat_urdf(model_path,args.output)
    model, collision, visual = pin.buildModelsFromUrdf(str(display_path),
        package_dirs=[str(model_path.parent.parent.parent)])
    names = list(model.names)[1:]
    if set(names) != set(data['joint_names']) or model.nq != len(names):
        raise ValueError('LIVE_MODEL_JOINT_LAYOUT')
    config = display_tool_config(data)
    recipe = config['demo']
    ready, views = queue.Queue(1), queue.Queue(8)
    model_status = LatestState()
    failures = []
    def server_worker():
        try:
            _server(read_state, args.trace_file, str(args.output), args.port,
                    ready, views, data, stop, model_status)
        except Exception as exc:
            failures.append(exc)
            stop.set()
    server = threading.Thread(target=server_worker, name='meshcat-http', daemon=True)
    server.start()
    viewer = None
    try:
        address = ready.get(timeout=15.)
        if isinstance(address, Exception):
            raise address
        zmq_url, web_url = address
        viewer = meshcat.Visualizer(zmq_url=zmq_url)
        import zmq
        viewer.window.zmq_socket.setsockopt(zmq.RCVTIMEO, 2000)
        viewer.window.zmq_socket.setsockopt(zmq.SNDTIMEO, 2000)
        vis = MeshcatVisualizer(model, collision, visual)
        vis.initViewer(viewer=viewer, open=False)
        vis.loadViewerModel(rootNodeName='snowbot')
        viewer['/Background'].set_property('top_color',[19,28,41])
        viewer['/Background'].set_property('bottom_color',[31,41,54])
        viewer['/Axes'].set_property('visible',False)
        ground = np.eye(4);ground[2,3]=recipe['ground']['z_in_base_m']
        viewer['/Grid'].set_transform(ground)
        mount = np.array(config['left_to_tool']).reshape(4,4)
        axis = np.asarray(recipe['shaft_axis_in_tool'])
        rear = (np.linalg.inv(mount) @ np.r_[recipe['grip_offset_m'],1.])[:3]
        ends = recipe['shaft_from_left_grip_m']
        ref = np.eye(3)[np.argmin(abs(axis))]
        tangent = np.cross(ref,axis); tangent /= np.linalg.norm(tangent)
        tf = np.eye(4);tf[:3,:3]=np.column_stack((tangent,axis,np.cross(tangent,axis)))
        tf[:3,3]=rear+axis*sum(ends)/2
        viewer['shovel/shaft'].set_object(geom.Cylinder(ends[1]-ends[0],recipe['shaft_radius_m']),
                                        geom.MeshLambertMaterial(color=0x996b40))
        viewer['shovel/shaft'].set_transform(tf)
        if recipe.get('geometry_kind')!='rod':
            viewer['shovel/tray'].set_object(geom.Box(recipe['pan_size_m']),geom.MeshLambertMaterial(color=0x4386ba))
            tf=np.eye(4);tf[:3,3]=recipe['pan_center_m'];viewer['shovel/tray'].set_transform(tf)
        for side,color in (('left',0x70aaff),('right',0x5bd6bb)):
            viewer[side+'_tip'].set_object(geom.Sphere(.018),geom.MeshLambertMaterial(color=color))
        start=time.monotonic();count=0;last_stamp=None;tool_enabled=True;last_save=0.
        print('SNOWBOT_WEB_READY '+web_url,flush=True)
        while not stop.is_set() and (not args.duration or time.monotonic()-start < args.duration):
            if not server.is_alive():
                raise RuntimeError('WEB_SERVER_EXITED')
            try:
                while True:
                    action=views.get_nowait()
                    if action['action']=='grid':viewer['/Grid'].set_property('visible',action['enabled'])
                    elif action['action']=='tool':tool_enabled=action['enabled']
            except queue.Empty:
                pass
            status='WAITING';tool_source='unavailable';tool_visible=False
            try:
                value=read_state()
                q=validate_state(value,time.time(),args.run_id,data['profile_hash'],names,
                                 args.runtime_mode,args.device_backend)
                if data.get('instance_id') and value.get('instance_id')!=data['instance_id']:
                    raise ValueError('LIVE_EXECUTOR_CHANGED')
                if not data.get('instance_id'):
                    data['instance_id']=value.get('instance_id')
                if last_stamp is not None and value['feedback_stamp']<last_stamp:
                    raise ValueError('LIVE_TIME_REVERSED')
                if value['feedback_stamp']!=last_stamp:
                    positions=pin.neutral(model)
                    for name in names:positions[model.joints[model.getJointId(name)].idx_q]=q[name]
                    vis.display(positions);pin.updateFramePlacements(model,vis.data)
                    for side in ('left','right'):
                        pose=vis.data.oMf[model.getFrameId(config[side+'_frame'])].homogeneous
                        viewer[side+'_tip'].set_transform(pose)
                    last_stamp=value['feedback_stamp'];count+=1
                pico=value.get('pico',{})
                left_pose=vis.data.oMf[model.getFrameId(config['left_frame'])].homogeneous
                tool_pose,tool_source=tool_display_pose(value,left_pose,mount,time.time())
                if tool_pose is not None:
                    viewer['shovel'].set_transform(tool_pose)
                tool_visible=bool(tool_enabled and tool_pose is not None)
                viewer['shovel'].set_property('visible',tool_visible)
                status=pico.get('phase','waiting')+' | '+pico.get('reason','')
            except (OSError,ValueError,KeyError) as exc:
                status=str(exc);viewer['shovel'].set_property('visible',False)
            if time.monotonic()-last_save>=1.:
                model_status.publish(dict(updated_at=time.time(),
                    url=web_url,applied_frames=count,status=status,robot_commands_published=0,
                    tool_source=tool_source,tool_visible=tool_visible))
                last_save=time.monotonic()
            time.sleep(.033)
    finally:
        if viewer:
            viewer.window.zmq_socket.close(linger=0)
        stop.set()
        server.join(timeout=3.)
        if server.is_alive():
            raise RuntimeError('WEB_SERVER_SHUTDOWN_TIMEOUT')
        if failures:
            raise RuntimeError('WEB_SERVER_FAILED') from failures[0]
