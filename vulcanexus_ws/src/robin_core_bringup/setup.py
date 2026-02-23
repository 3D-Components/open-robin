import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robin_core_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes', 'welding_table', 'visual'), glob('meshes/welding_table/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'welding_table', 'collision'), glob('meshes/welding_table/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milaipc',
    maintainer_email='jayant@mil-as.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
