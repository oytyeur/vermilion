from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='specific_object_searcher_py',
            executable='specific_object_searcher_node',
            name='specific_object_searcher_node',
            parameters=['config/params.yaml'],
            output='screen'
        )
    ])