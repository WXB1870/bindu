from setuptools import setup, find_packages

setup(
    name='bindu_planning', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_planning']),
                ('share/bindu_planning', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
)
