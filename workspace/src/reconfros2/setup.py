import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rl4greenros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds/', ['worlds/rl4greenros_room.world']),
        ('share/' + package_name + '/worlds/', ['worlds/rl4greenros_large_room.world']),
        ('share/' + package_name + '/rosbag/', ['rosbag/metadata.yaml']),
        ('share/' + package_name + '/rosbag/', ['rosbag/rosbag2_2024_05_03-20_26_41_0.db3']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michel Albonico',
    maintainer_email='michelalbonico@utfpr.edu.br',
    description='Package used to bring up the RL4GreenROS experimental environment.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_navigation_goals = rl4greenros.multi_navigation_goals:main',
            'single_navigation_goal = rl4greenros.single_navigation_goal:main',
            'nav_to_pose = rl4greenros.nav_to_pose:main',
            'nav_to_pose_action = rl4greenros.nav_to_pose_action:main',
            'initial_pose = rl4greenros.initial_pose:main',
            'add_obstacle = rl4greenros.add_obstacle:main',
            'spawn_robot = rl4greenros.respawn_gazebo:spawn_entity',
            'respawn_world = rl4greenros.respawn_gazebo:reset_world',
            'reset_world = rl4greenros.gazebo_reset_world:main',
        ],
    },
)
