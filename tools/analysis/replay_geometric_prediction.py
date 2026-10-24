"""Build and replay an offline rigid-grasp prediction candidate.

Uses the existing execution curve/stop primitives. Generated FK covers only the
two end-frame chains. This is not connected to runtime or any device backend.
The input is the native replay format followed by captured joint positions.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]

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
    return '#pragma once\n#include <array>\nnamespace prediction_model {\nstruct Node{int parent,index;bool prismatic;std::array<double,16> fixed;std::array<double,3> axis;};\ninline constexpr int dofs='+str(len(names))+',left='+str(world[left])+',right='+str(world[right])+';\ninline const std::array<Node,'+str(len(nodes))+'> nodes={{'+','.join(nodes)+'}};\n}\n'

def eigen_include(explicit=None):
    if explicit:return Path(explicit)
    candidates=[Path('/usr/include/eigen3'),Path('/usr/local/include/eigen3')]
    candidates+=list(Path(sys.prefix).glob('lib/python*/site-packages/cmeel.prefix/include/eigen3'))
    return next((p for p in candidates if (p/'Eigen/Dense').is_file()),None)

def build(output,urdf,names,left,right,eigen=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    include=eigen_include(eigen)
    compiler=shutil.which('clang++') or shutil.which('g++')
    if not include or not compiler:raise RuntimeError('Existing C++17 compiler and Eigen headers required; no automatic installation')
    (output/'prediction_model.hpp').write_text(model_header(urdf,names,left,right))
    core=ROOT/'src/control/bindu_execution/native';source=Path(__file__).with_name('geometric_prediction.cpp');exe=output/'geometric_prediction'
    files=[source,output/'prediction_model.hpp',*[core/n for n in ['interpolation.cpp','interpolation.hpp','adaptive_stream.cpp','adaptive_stream.hpp','curve_math.hpp']]]
    signature={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};manifest=output/'build.json'
    if not exe.exists() or not manifest.exists() or json.loads(manifest.read_text())!=signature:
        subprocess.run([compiler,'-O2','-std=c++17','-I',str(output),'-I',str(core),'-I',str(include),str(source),str(core/'interpolation.cpp'),str(core/'adaptive_stream.cpp'),'-o',str(exe)],check=True)
        manifest.write_text(json.dumps(signature,indent=2)+'\n')
    return exe

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--urdf',required=True);p.add_argument('--names',required=True,help='JSON array of active joint names');p.add_argument('--left-frame',required=True);p.add_argument('--right-frame',required=True);p.add_argument('--output',required=True);p.add_argument('--input');p.add_argument('--eigen-include');p.add_argument('--strategy',choices=['A','project','geometry','adaptive','check'],default='geometry');p.add_argument('--independent',action='store_true');a=p.parse_args()
    exe=build(a.output,a.urdf,json.loads(Path(a.names).read_text()),a.left_frame,a.right_frame,a.eigen_include)
    if a.input:
        out=Path(a.output);result=subprocess.run([str(exe),a.strategy,str(int(not a.independent))],input=Path(a.input).read_text(),text=True,capture_output=True)
        (out/(a.strategy+'.csv')).write_text(result.stdout);(out/(a.strategy+'.events')).write_text(result.stderr)
        if result.returncode:raise RuntimeError(result.stderr[-2000:])
        print(json.dumps(dict(strategy=a.strategy,reference_only=True,hardware_commands=False,returncode=result.returncode)))

if __name__=='__main__':main()
