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
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Intent-to-skill mission controller for the ROBIN welding HRI PoC',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_supervisor_node = '
            'welding_supervisor.welding_supervisor_node:main',
        ],
    },
)
