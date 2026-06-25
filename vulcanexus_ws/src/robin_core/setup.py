import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robin_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='Core planning, calibration, data and sensor processing for ROBIN',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robin_planner_node = robin_core.robin_planner_node:main',
            'experiment_node = robin_core.experiment_node:main',
            'weld_data_node = robin_core.weld_data_node:main',
            'progression_node = robin_core.progression_node:main',
            'process_data_node = robin_core.process_data_node:main',
            'tcp_manager_node = robin_core.tcp_manager_node:main',
            'plate_markers_node = robin_core.plate_markers_node:main',
            'view_profile_node = robin_core.view_profile_node:main',
            'sim_clock_monitor_node = robin_core.sim_clock_monitor_node:main',
            'telemetry_aggregator_node = robin_core.telemetry_aggregator_node:main',
        ],
    },
)
