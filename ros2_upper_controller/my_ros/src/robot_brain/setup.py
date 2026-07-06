from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_brain'


def glob_files(pattern: str):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob_files('config/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Task manager and planning helpers for the cleaner robot stack.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'task_manager_node = robot_brain.task_manager_node:main',
            'robot_state_aggregator = robot_brain.robot_state_aggregator:main',
        ],
    },
)
