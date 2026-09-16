from setuptools import setup, find_packages
from glob import glob
setup(name='bindu_core', version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages',['resource/bindu_core']),('share/bindu_core',['package.xml']),('share/bindu_core/config',glob('config/*.json')),('share/bindu_core/launch',glob('launch/*.py'))], install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, entry_points={'console_scripts':[]})
