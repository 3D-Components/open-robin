import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robin_hardware_fronius'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'weld_torch', 'visual'), glob('meshes/weld_torch/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'weld_torch', 'collision'), glob('meshes/weld_torch/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milaipc',
    maintainer_email='jayant@mil-as.no',
    description='Fronius welding coordinator - orchestrates welding sequences',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_coordinator = robin_hardware_fronius.welding_coordinator:main',
        ],
    },
)
