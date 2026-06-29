import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robin_description'


def mesh_data_files():
    """Install every file under meshes/ preserving its relative directory."""
    grouped = {}
    for path in glob('meshes/**/*', recursive=True):
        if os.path.isfile(path):
            dest = os.path.join('share', package_name, os.path.dirname(path))
            grouped.setdefault(dest, []).append(path)
    return [(dest, files) for dest, files in grouped.items()]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + mesh_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Haas, Virgilio Gomez, Jayant Singh',
    maintainer_email='daniel.haas@3d-components.co, virgilio.gomez@3d-components.co, jayant@mil-as.no',
    description='Shared URDF/xacro description and meshes for the ROBIN welding cell '
                '(UR10e + Fronius torch + Garmo profilometer + table). Hardware-agnostic.',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
