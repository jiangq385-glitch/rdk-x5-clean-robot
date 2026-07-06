from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'


def files_only(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'urdf'), files_only('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), files_only('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), files_only('rviz/*')),
        (os.path.join('share', package_name, 'config'), files_only('config/*')),
        (os.path.join('share', package_name, 'launch'), files_only('launch/*')),
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
        ],
    },
)
