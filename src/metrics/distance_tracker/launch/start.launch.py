from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='distance_tracker',
            executable='distance_tracker_exec',
            name='distance_tracker',
            output='screen',
            parameters=[
                {'odom_topic': '/odom'}
            ]
        )
    ])
