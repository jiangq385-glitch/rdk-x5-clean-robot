from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mipi_yolo_node = robot_vision.mipi_yolo_node:main',
            'laser_range_node = robot_vision.laser_range_node:main',
            'object_on_plane_node = robot_vision.object_pose_node:main',
            'object_pose_node = robot_vision.object_pose_node:main',
            'plane_calibration_node = robot_vision.plane_calibration_node:main',
        ],
    },
)
