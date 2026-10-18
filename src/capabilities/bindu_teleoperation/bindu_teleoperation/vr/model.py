"""Local visual URDF assets and read-only Vuer joint display."""
import hashlib
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from .mapping import OPENXR_TO_ROBOT


def prepare_model(source, static_root, tool_config=None, *, short_rod=False):
    source, static_root = Path(source), Path(static_root)
    tree = ET.parse(source)
    package = source.parent.parent
    assets = static_root/'model-assets'
    assets.mkdir()
    copied = set()
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:continue
            uri = mesh.get('filename')
            prefix = f'package://{package.name}/'
            path = package/uri.removeprefix(prefix) if uri.startswith(prefix) else Path(uri)
            if not path.is_absolute():raise ValueError('UNRESOLVED_VUER_MESH:'+uri)
            raw = path.read_bytes()
            if path.suffix.lower()=='.stl' and len(raw)==84 and raw[80:]==b'\x00'*4:
                link.remove(visual)
                continue
            name = hashlib.sha256(str(path).encode()).hexdigest()[:16]+path.suffix.lower()
            if name not in copied:
                shutil.copyfile(path,assets/name)
                copied.add(name)
            mesh.set('filename','/static/model-assets/'+name)
    tree.write(static_root/'snowbot.urdf',encoding='utf-8',xml_declaration=True)
    if tool_config:
        import json,math
        cfg=json.loads(Path(tool_config).read_text())
        if short_rod:
            from ..simulation_fixture import short_rod_config
            cfg=short_rod_config(cfg)
        recipe=cfg['demo']
        mount=np.array(cfg['left_to_tool']).reshape(4,4);r=mount[:3,:3]
        link=ET.SubElement(tree.getroot(),'link',name='bindu_virtual_tool')
        joint=ET.SubElement(tree.getroot(),'joint',name='bindu_virtual_tool_mount',type='fixed')
        ET.SubElement(joint,'parent',link=cfg['left_frame']);ET.SubElement(joint,'child',link='bindu_virtual_tool')
        angles=[math.atan2(r[2,1],r[2,2]),math.atan2(-r[2,0],math.hypot(r[0,0],r[1,0])),math.atan2(r[1,0],r[0,0])]
        ET.SubElement(joint,'origin',xyz=' '.join(map(str,mount[:3,3])),rpy=' '.join(map(str,angles)))
        axis=np.array(recipe['shaft_axis_in_tool']);ends=recipe['shaft_from_left_grip_m']
        rear=(np.linalg.inv(mount)@np.r_[recipe['grip_offset_m'],1.])[:3]
        center=rear+axis*sum(ends)/2
        for name,xyz,rpy,color in [('shaft',center,[0.,math.atan2(axis[0],axis[2]),0.],'0.60 0.42 0.25 1'),
                ('pan',recipe['pan_center_m'],[0.,0.,0.],'0.26 0.53 0.73 1')]:
            if name=='pan' and recipe.get('geometry_kind')=='rod':continue
            visual=ET.SubElement(link,'visual',name=name)
            ET.SubElement(visual,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
            geo=ET.SubElement(visual,'geometry')
            if name=='shaft':ET.SubElement(geo,'cylinder',radius=str(recipe['shaft_radius_m']),length=str(ends[1]-ends[0]))
            else:ET.SubElement(geo,'box',size=' '.join(map(str,recipe['pan_size_m'])))
            material=ET.SubElement(visual,'material',name='virtual_'+name);ET.SubElement(material,'color',rgba=color)
        tree.write(static_root/'snowbot-tool.urdf',encoding='utf-8',xml_declaration=True)
    return '/static/snowbot.urdf'


def render_model(view):
    from vuer.schemas import Urdf, Group, AmbientLight, DirectionalLight
    joints = view.get('model_joints')
    if not view.get('model_url') or not joints:return None
    # A separate simulation model, sharing operator-forward two metres ahead.
    # It is not registered to the physical room or a robot camera.
    matrix = OPENXR_TO_ROBOT.T.copy()
    matrix[:3,3] = [0.,.2,-2.]
    url=view.get('model_tool_url') if view.get('load_state')=='held' else None
    return Group(Urdf(src=url or view['model_url'],jointValues=joints,key='snowbotURDF',
        matrix=matrix.ravel(order='F').tolist(),parseVisual=True,parseCollision=False,
        materialType='standard',material={'color':'#b4c8bf','roughness':.65,'metalness':.15}),
        AmbientLight(intensity=.75,key='snowbotAmbient'),
        DirectionalLight(intensity=1.5,position=[1.,3.,1.],key='snowbotLight'),key='snowbotLiveModel')
