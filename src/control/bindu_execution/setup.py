from setuptools import Extension, find_packages, setup

setup(
    name='bindu_execution', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_execution']),
                ('share/bindu_execution', ['package.xml'])],
    ext_modules=[Extension('bindu_execution._native',
        sources=['native/python_bindings.cpp', 'native/interpolation.cpp', 'native/adaptive_stream.cpp'],
        depends=['native/interpolation.hpp', 'native/adaptive_stream.hpp', 'native/curve_math.hpp'],
        language='c++', extra_compile_args=['-std=c++17', '-O2', '-Wall', '-Wextra'])],
    install_requires=['setuptools'], zip_safe=True,
)
