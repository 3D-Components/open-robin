import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robin_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'meshes', 'welding_table', 'visual'), glob('meshes/welding_table/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'welding_table', 'collision'), glob('meshes/welding_table/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='Bringup package for the ROBIN welding robot system',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lifecycle_manager = robin_bringup.lifecycle_manager_node:main',
        ],
    },
)
