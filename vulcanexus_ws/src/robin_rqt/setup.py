from setuptools import setup, find_packages

package_name = 'robin_rqt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/config', ['config/laser_profile.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROBIN Team',
    maintainer_email='operator@robin.local',
    description='RQT Operator Panel for ROBIN welding system',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
