from setuptools import setup, find_packages
from glob import glob
setup(name='bindu_teleoperation', version='0.1.0', packages=find_packages(),
      package_data={'bindu_teleoperation.vr': ['client_timing.js']},
      data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_teleoperation']),
                  ('share/bindu_teleoperation', ['package.xml']),
                  ('share/bindu_teleoperation/models', glob('models/*'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True)
