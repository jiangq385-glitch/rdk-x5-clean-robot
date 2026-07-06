import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_test'


def glob_files(pattern: str):
 return [p for p in glob(pattern) if os.path.isfile(p)]


setup(
 name=package_name,
 version='0.0.0',
 packages=find_packages(exclude=['test']),
 data_files=[
 ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml']),
 (os.path.join('share', package_name, 'launch'), glob_files('launch/*')),
 ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='sunrise',
 maintainer_email='3451912207@qq.com',
 description='End-to-end clean-area test flow for navigation, USB YOLO, and Doubao vision LLM.',
 license='TODO: License declaration',
 extras_require={'test': ['pytest']},
 entry_points={
 'console_scripts': [
 'clean_table_test_node = robot_test.clean_table_test_node:main',
 ],
 },
)
