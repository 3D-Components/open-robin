from setuptools import find_packages, setup

package_name = 'welding_http_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='HTTP bridge between ROBIN React dashboard and ROS2 /intents topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_http_bridge_node = '
            'welding_http_bridge.welding_http_bridge_node:main',
        ],
    },
)
