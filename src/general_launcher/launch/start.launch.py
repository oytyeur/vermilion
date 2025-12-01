import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

def add_pointcloud_to_laserscan_node() -> Node:
    """Create pointcloud to laserscan conversion node"""

    return Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='go2_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed'),
            ('scan', 'scan'),
        ],
        parameters=[{
            # 'target_frame': 'odom',
            'min_height': 0.12,   
            'max_height': 1.0,
            'angle_increment': 0.005
        }],
        output='screen',
    )


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Combine all elements
    launch_entities = [
        add_pointcloud_to_laserscan_node(),
        Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"]),
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             os.path.join(get_package_share_directory('slam_toolbox'),
        #                         'launch', 'online_async_launch.py')
        #         ]),
        #         launch_arguments={
        #             'use_sim_time': use_sim_time,
        #             'odom_frame': 'odom',
        #             'map_frame': 'map',
        #             'base_frame': 'base_link'
        #         }.items(),
        # )
    ]
    
    return LaunchDescription(launch_entities)