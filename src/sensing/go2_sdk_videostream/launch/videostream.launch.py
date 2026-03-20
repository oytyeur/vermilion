from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_sdk_videostream',
            executable='go2_sdk_videostream_node',
            name='frame_publisher',
            output='screen',
            parameters=[{
                # 'network_interface': 'enx00e04c6803d4',
                'network_interface': 'enp2s0',
                # 'network_interface': 'wlp3s0',
                'publish_rate': 15.0
            }]
        )
    ])