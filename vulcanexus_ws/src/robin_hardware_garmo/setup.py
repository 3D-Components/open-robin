from setuptools import find_packages, setup

package_name = 'robin_hardware_garmo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so they are available via ament/share/<pkg>/launch
        ('share/' + package_name + '/launch', [
            'launch/sensor.launch.py',
        ]),
        # Install rviz configurations
        ('share/' + package_name + '/rviz', [
            'rviz/debug_garmo.rviz',
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
    maintainer='Jayant',
    maintainer_email='jayant@mil-as.no',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_data = robin_hardware_garmo.sensor_data:main',
            'sensor_cmd = robin_hardware_garmo.sensor_cmd:main',
        ],
    },
)
