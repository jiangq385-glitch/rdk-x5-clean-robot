from setuptools import find_packages, setup

package_name = 'robot_skills'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Skill action servers for the cleaner robot stack.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pick_object_server = robot_skills.pick_object_server:main',
            'place_object_server = robot_skills.place_object_server:main',
            'navigate_to_object_server = robot_skills.navigate_to_object_server:main',
            'clean_area_server = robot_skills.clean_area_server:main',
            'return_home_server = robot_skills.return_home_server:main',
            'mock_servers = robot_skills.mock_servers:main',
        ],
    },
)
