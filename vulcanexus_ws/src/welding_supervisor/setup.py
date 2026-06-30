from setuptools import find_packages, setup

package_name = 'welding_supervisor'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='Intent-to-skill mission controller for the ROBIN welding HRI PoC',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_supervisor_node = '
            'welding_supervisor.welding_supervisor_node:main',
        ],
    },
)
