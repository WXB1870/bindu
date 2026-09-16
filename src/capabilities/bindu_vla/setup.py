from setuptools import setup, find_packages

setup(
    name='bindu_vla', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bindu_vla']),
                ('share/bindu_vla', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
)
