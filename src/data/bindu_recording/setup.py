from setuptools import find_packages, setup

setup(
    name='bindu_recording', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_recording']),
                ('share/bindu_recording', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
)
