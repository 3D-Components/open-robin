import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robin_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='ROBIN welding process simulation package',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_opcua_bridge = robin_simulation.fake_opcua_bridge_node:main',
            'sim_garmo_sensor = robin_simulation.sim_garmo_sensor_node:main',
        ],
    },
)
