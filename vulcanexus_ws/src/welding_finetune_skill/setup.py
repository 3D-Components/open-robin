from setuptools import find_packages, setup

package_name = 'welding_finetune_skill'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Mock model fine-tuning skill for the ROBIN dashboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'welding_finetune_skill_node = '
            'welding_finetune_skill.welding_finetune_skill_node:main',
        ],
    },
)
