from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robin_hardware_ur'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include URDF/Xacro files (recursive to catch all layouts, e.g., robin_scene.urdf.xacro)
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/**/*.xacro', recursive=True) +
            glob('urdf/**/*.urdf', recursive=True)
        ),

        # Include config files (if present)
        (os.path.join('share', package_name, 'config'), glob('config/**/*', recursive=True)),

        # Include launch files (*.launch.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include RViz configs (if present)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in (
            glob('meshes/**/*.stl', recursive=True) +
            glob('meshes/**/*.obj', recursive=True) +
            glob('meshes/**/*.mtl', recursive=True)
        )
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
        ],
    },
)
