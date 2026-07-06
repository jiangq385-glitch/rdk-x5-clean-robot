from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'robot_nav2'


def _glob_files(pattern: str):
    return [p for p in glob(pattern) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), _glob_files('launch/*')),
        (os.path.join('share', package_name, 'config'), _glob_files('config/*')),
        (os.path.join('share', package_name, 'maps'), _glob_files('maps/*')),
        (os.path.join('share', package_name, 'behavior_trees'), _glob_files('behavior_trees/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'way_point = robot_nav2.way_point:main',
            'auto_relocalize = robot_nav2.auto_relocalize:main',
        ],
    },
)
