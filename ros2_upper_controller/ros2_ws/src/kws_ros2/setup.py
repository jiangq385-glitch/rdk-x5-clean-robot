from setuptools import find_packages, setup

package_name = 'kws_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/kws.launch.py']),
        ('lib/' + package_name, ['scripts/kws_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='3451912207@qq.com',
    description='Lightweight wake-word front-end for ROS2 voice pipeline',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kws_node = kws_ros2.kws_node:main',
            'wake_manager = kws_ros2.wake_manager:main',
        ],
    },
)
