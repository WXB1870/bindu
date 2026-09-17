from setuptools import setup, find_packages
from glob import glob
setup(name='bindu_kinematics', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_kinematics']),
                  ('share/bindu_kinematics', ['package.xml']),
                  ('share/bindu_kinematics/models', glob('models/*'))],
      install_requires=['setuptools'], zip_safe=True)
