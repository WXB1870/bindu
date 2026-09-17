from setuptools import setup, find_packages
from glob import glob
setup(name='bindu_teleoperation', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_teleoperation']),
                  ('share/bindu_teleoperation', ['package.xml']),
                  ('share/bindu_teleoperation/models', glob('models/*'))],
      install_requires=['setuptools'], zip_safe=True)
