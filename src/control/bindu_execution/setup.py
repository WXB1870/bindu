from os.path import relpath
from pathlib import Path
from setuptools import Extension, find_packages, setup

# colcon symlink-install runs this symlinked script from its build directory.
# Keep paths relative for setuptools archives, but resolve the real source root.
native = Path(relpath(Path(__file__).resolve().parent / 'native'))

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
