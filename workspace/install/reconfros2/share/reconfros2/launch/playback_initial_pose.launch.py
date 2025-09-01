import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rosbag_folder = os.path.join(
        get_package_share_directory('reconfros2'),
        'rosbag'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '--topics=\"/initialpose\"', rosbag_folder],
            output='screen',
            shell=True
        )
    ])
