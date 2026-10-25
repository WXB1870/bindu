"""Deterministic URDF-to-native prediction model generation; no ROS or devices."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def model_header(urdf,names,left,right):
    if len(names)!=len(set(names)) or not names:raise ValueError('joint names must be unique and nonempty')
    xml=ET.parse(urdf).getroot();joints=xml.findall('joint');children={j.find('child').get('link'):j for j in joints}
    movable={j.get('name') for j in joints if j.get('type')!='fixed'}
    if any(n not in movable for n in names):raise ValueError('active joint is unknown or fixed')
    needed=set();roots=set()
    for frame in [left,right]:
        if xml.find(f"link[@name='{frame}']") is None:raise ValueError('unknown end frame')
        seen=set()
        while frame in children:
            if frame in seen:raise ValueError('cyclic kinematic tree')
            seen.add(frame)
            needed.add(frame);frame=children[frame].find('parent').get('link')
        roots.add(frame)
    if len(roots)!=1:raise ValueError('end frames must share one root')
    world={roots.pop():0};pending=[j for j in joints if j.find('child').get('link') in needed];nodes=[]
    while pending:
        old=len(pending)
        for joint in pending[:]:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in world:continue
            kind=joint.get('type');name=joint.get('name');origin=joint.find('origin')
            if kind not in ['fixed','revolute','continuous','prismatic']:raise ValueError('unsupported joint type')
            if kind!='fixed' and name not in names:raise ValueError('moving ancestor missing from joint layout')
            xyz=list(map(float,origin.get('xyz','0 0 0').split())) if origin is not None else [0.,0.,0.]
            rpy=list(map(float,origin.get('rpy','0 0 0').split())) if origin is not None else [0.,0.,0.]
            cr,cp,cy=map(math.cos,rpy);sr,sp,sy=map(math.sin,rpy)
            fixed=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,xyz[0],sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,xyz[1],-sp,cp*sr,cp*cr,xyz[2],0,0,0,1]
            axis=joint.find('axis');axis=list(map(float,axis.get('xyz','1 0 0').split())) if axis is not None else [1.,0.,0.]
            if kind=='fixed':axis=[1.,0.,0.]
            norm=math.sqrt(sum(x*x for x in axis))
            if not norm or not all(math.isfinite(x) for x in [*axis,*fixed]):raise ValueError('invalid joint geometry')
            axis=[x/norm for x in axis]
            nodes.append('{'+f'{world[parent]},{names.index(name) if name in names else -1},{str(kind=="prismatic").lower()},'+'{'+','.join(format(x,'.17g') for x in fixed)+'},{'+','.join(format(x,'.17g') for x in axis)+'}}')
            world[child]=len(nodes);pending.remove(joint)
        if len(pending)==old:raise ValueError('invalid or cyclic kinematic tree')
    metadata='inline constexpr char source_sha256[]='+json.dumps(hashlib.sha256(Path(urdf).read_bytes()).hexdigest())+';\ninline constexpr char left_frame[]='+json.dumps(left)+';\ninline constexpr char right_frame[]='+json.dumps(right)+';\n'
    return '#pragma once\n#include "rigid_prediction.hpp"\nnamespace prediction_model {\nusing Node=bindu::execution::rigid::Node;\ninline constexpr int dofs='+str(len(names))+',left='+str(world[left])+',right='+str(world[right])+';\ninline const std::array<Node,'+str(len(nodes))+'> nodes={{'+','.join(nodes)+'}};\n'+metadata+'}\n'
