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

from bindu_kinematics.model_codegen import model_header

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
    files=[source,output/'prediction_model.hpp',*[core/n for n in ['interpolation.cpp','interpolation.hpp','adaptive_stream.cpp','adaptive_stream.hpp','curve_math.hpp','rigid_prediction.hpp']]]
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
