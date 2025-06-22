from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elrs_teleop',
            executable='elrs_teleop_node',
            name='elrs_teleop',
            output='screen',
            parameters=[]
        )
    ])
