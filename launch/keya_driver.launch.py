import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_keya_driver',
            executable='keya_driver',
            name='keya_driver_node',
            # parameters=[('/home/fib02204/ros2_ws/src/ros2_keya_driver/config/params.yaml')],
            output='screen'),

    ])
