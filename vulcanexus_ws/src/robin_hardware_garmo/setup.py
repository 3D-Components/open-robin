from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robin_hardware_garmo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Install launch files so they are available via ament/share/<pkg>/launch
        ('share/' + package_name + '/launch', [
            'launch/sensor.launch.xml',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/garmo_garline_macro.xacro',
        ]),
        ('share/' + package_name + '/meshes' + '/garmo_garline', [
            'meshes/garmo_garline/garmo_garline_visual.obj',
            'meshes/garmo_garline/garmo_garline_visual.mtl',
            'meshes/garmo_garline/garmo_garline_collision.stl',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='Garmo laser profilometer driver for the ROBIN platform',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = robin_hardware_garmo.sensor_node:main',
        ],
    },
)
