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
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed'),
            ('scan', 'scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': 0.12,   
            'max_height': 1.5,
            'angle_increment': 0.005
        }],
        output='screen',
    )


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Combine all elements
    launch_entities = [
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='go2_rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('general_launcher'), 'config', 'basic_rviz_conf.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),


        add_pointcloud_to_laserscan_node(),


        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                launch_arguments={
                    'slam_params_file': os.path.join(get_package_share_directory('general_launcher'), 'config', 'mapper_params_online_async.yaml'),
                    'use_sim_time': use_sim_time,
                }.items(),
            ),


        # Node(
        #     package='pc2_values_demonstrator',
        #     executable='pc2_values_demonstrator_exec',
        #     name='pc2_values_demonstrator_node',
        #     remappings=[
        #         ('input', '/utlidar/cloud_deskewed'),
        #     ]

        # ),


        Node(
            package='odom_baselink_tf_publisher',
            executable='odom_baselink_tf_publisher_exec',
            name='odom_baselink_tf_publisher_node',
            remappings=[
                ('input', '/utlidar/robot_pose'),
            ]
        )
    ]
    
    return LaunchDescription(launch_entities)