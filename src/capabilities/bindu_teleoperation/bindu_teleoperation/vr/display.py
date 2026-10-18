"""Small Vuer 0.0.60 scene; rendered off the controller event loop."""
import numpy as np
from .mapping import OPENXR_TO_ROBOT, pose_matrix


def viewer_matrix(pose):
    # Express robot-base poses in Y-up viewer coordinates, one metre in front.
    # This is a diagram, not registration to the operator's physical room.
    result = OPENXR_TO_ROBOT.T @ pose_matrix(pose)
    result[2, 3] -= 1.
    return result.ravel(order='F').tolist()


def status_lines(view):
    if view.get('connectivity'):
        from .connectivity import status_lines as connectivity_lines
        return connectivity_lines(view)
    if view.get('runtime_mode')=='real':
        return [('BINDU | REAL | BIMANUAL POSITION','#f1f5f9'),
                ('Session status: check ROS feedback','#ffffff'),
                ('Squeeze: relative follow | Y: stop','#cbd5e1'),
                ('Triggers / X / A / B / sticks: stop only','#fbbf24'),
                ('Gripper controls unavailable in this entry','#cbd5e1'),
                ('Startup: no automatic preparation','#cbd5e1')]
    if view.get('pico'):
        from .hud import hud_spec
        spec = hud_spec(view)
        lines = [(' · '.join(spec[k] for k in ('context','status','grip','action')),'#e5eeea')]
        if spec['title']:
            lines += [(spec['title'],'#f0c476'),(spec['detail'],'#a7bbb0')]
        return lines
    lines = [('BINDU | SINGLE ARM SIMULATION | ' + view.get('side', 'left'), '#f1f5f9'),
             ('Mode: ' + view['mode'] + ' (requested)', '#ffffff'),
             ('Reason: ' + view['reason'], '#fbbf24'),
             ('CYAN = requested target | ORANGE = feedback FK', '#7dd3fc'),
             ('Axes RGB = local XYZ | robot_base | metres', '#cbd5e1'),
             ('Squeeze: release to pause; hold 1s to resume', '#cbd5e1')]
    error = view.get('error')
    lines.append(('Error: %.1f mm / %.1f deg' % (error['position_m']*1000, np.degrees(error['rotation_rad']))
                  if error else 'Error: -- (fresh target and feedback required)', '#ffffff'))
    return lines


def render(view):
    from vuer.schemas import Group, CoordsMarker, Sphere, Plane, SceneElement
    if view.get('pico') and view.get('runtime_mode')!='real':
        from .hud import render_hud
        from .model import render_model
        model=render_model(view)
        children=([model] if model is not None else [])+[render_hud(view)]
        return Group(*children,key='teleopFeedback',
            userData={'mode':view['mode'],'reason':view['reason'],'simulated':True})

    # These components exist in Vuer 0.0.60's frontend. Its Python package
    # exposes HUDPlane through Image and does not yet wrap the Text component.
    class Text(SceneElement):
        tag = 'Text'

    class StatusHUD(SceneElement):
        tag = 'HUDPlane'
    lines = status_lines(view)
    panel = [Plane(key='statusBackground', args=[1.005, 1.005], position=[0., 0., .001],
                   materialType='basic', material={'color': '#101b2a', 'depthTest': False})]
    spacing=.90/max(7,len(lines))
    for i, (line, color) in enumerate(lines):
        panel.append(Text(line, key='statusLine'+str(i), font='/static/display.ttf',
            fontSize=.07*min(1.,7/len(lines)), anchorX='left', anchorY='top', color=color,
            position=[-.47, .44-i*spacing, .002], scale=[1/3.2, 1., 1.],
            maxWidth=3., overflowWrap='break-word', depthTest=False))
    children = [StatusHUD(*panel, key='teleopStatus', distanceToCamera=1.,
                          height=.30, aspect=3.2, position=[0., -.30, 0.])]
    for name, color, radius in (('target', '#22d3ee', .022), ('measured', '#fb923c', .012)):
        pose = view.get(name)
        if pose is not None:
            children.append(Group(
                CoordsMarker(key=name+'Axes', scale=.12),
                Sphere(key=name+'Point', args=[radius, 16, 12], materialType='basic', material={'color': color, 'wireframe': name == 'target'}),
                key=name+'Pose', matrix=viewer_matrix(pose)))
    if not view.get('connectivity'):
        children.append(Group(CoordsMarker(key='robotBaseAxes', scale=.2),
                              key='robotBase', matrix=viewer_matrix(np.eye(4))))
    return Group(*children, key='teleopFeedback',
                 userData={'mode': view['mode'], 'reason': view['reason'], 'simulated': view.get('runtime_mode')!='real'})
