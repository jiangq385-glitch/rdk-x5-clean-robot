import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_mqtt_bridge'


def glob_files(pattern: str):
    return [p for p in glob(pattern) if os.path.isfile(p)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob_files('launch/*')),
        (os.path.join('share', package_name, 'config'), glob_files('config/*')),
        (os.path.join('share', package_name, 'systemd'), glob_files('systemd/*')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='3451912207@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = robot_mqtt_bridge.mqtt_bridge_node:main',
        ],
    },
)
