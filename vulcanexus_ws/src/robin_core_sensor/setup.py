from setuptools import find_packages, setup

package_name = 'robin_core_sensor'

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
            'launch/view_sensor.launch.py',
            'launch/process_data.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayant',
    maintainer_email='jayant@mil-as.no',
    description='Core sensor utilities and ROS 2 nodes for the ROBIN platform - includes processing and calibration tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_data = robin_core_sensor.process_data:main',
        ],
    },
)
