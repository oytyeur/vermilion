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

    launch_entities = [


        
        Node(
            package='people_counter',
            executable='people_counter_exec',
            name='people_counter_node',
            remappings=[
                ('input', '/detections'),
                ('output', '/person_counter'),
                ('raw_frame', '/camera/raw_frame')
            ],
            parameters=[{'conf_threshold': 0.2}]
        )
    ]
    
    return LaunchDescription(launch_entities)