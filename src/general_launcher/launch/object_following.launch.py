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
    """Generate the launch description for Go2 robot system"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Combine all elements
    launch_entities = [
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('general_launcher'), 'config', 'video_detection.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        IncludeLaunchDescription( # лаунчер с детецией
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('go2_sdk_videostream'),
                            'launch', 'videostream.launch.py')
            ]),
        ),


        IncludeLaunchDescription( # лаунчер с детецией
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('yolo_detector'),
                            'launch', 'redball_search.launch.py')
            ]),
        ),

        IncludeLaunchDescription( # лаунчер с детецией
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('specific_object_follower'),
                            'launch', 'start.launch.py')
            ]),
        ),
    ]
    
    return LaunchDescription(launch_entities)