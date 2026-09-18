from pathlib import Path
from setuptools import Extension, find_packages, setup

# colcon symlink-install runs this symlinked script from its build directory.
# Archives use relative paths. Editable builds outside the source directory
# need absolute paths: some distutils versions mishandle ../ in object paths.
source_root = Path(__file__).resolve().parent
native = Path('native') if Path.cwd().resolve() == source_root else source_root / 'native'

setup(
    name='bindu_execution', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_execution']),
                ('share/bindu_execution', ['package.xml'])],
    ext_modules=[Extension('bindu_execution._native',
        sources=[str(native / name) for name in ('python_bindings.cpp', 'interpolation.cpp', 'adaptive_stream.cpp')],
        depends=[str(native / name) for name in ('interpolation.hpp', 'adaptive_stream.hpp', 'curve_math.hpp')],
        language='c++', extra_compile_args=['-std=c++17', '-O2', '-Wall', '-Wextra'])],
    install_requires=['setuptools'], zip_safe=True,
)
