from setuptools import setup

package_name = 'robin_hardware_fronius'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROBIN Team',
    maintainer_email='maintainers@3d-components.example',
    description='Publishes dummy Fronius welding measurements for integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_data = robin_hardware_fronius.welding_data:main',
        ],
    },
)

