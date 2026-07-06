import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_arm_control'


def files_only(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), files_only('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Lightweight arm control stack without URDF dependency.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_controller_node = my_arm_control.arm_controller_node:main',
            'joint_state_node = my_arm_control.joint_state_node:main',
            'joint_cli = my_arm_control.joint_cli:main',
            'vision_pick_node = my_arm_control.vision_pick_node:main',
        ],
    },
)
