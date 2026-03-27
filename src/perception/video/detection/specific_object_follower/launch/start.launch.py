import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

def generate_launch_description():

    # Combine all elements
    launch_entities = [

        Node(
            package='specific_object_follower',
            executable='specific_object_follower_exec',
            name='specific_object_follower_node',
            remappings=[
                ('input', '/detections')
            ],
            parameters=[{'conf_threshold': 1.0}] # для карты лучше выставить побольше (10 сек было хорошо)
        ),
    ]
    
    return LaunchDescription(launch_entities)