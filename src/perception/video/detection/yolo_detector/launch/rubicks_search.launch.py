from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('yolo_detector'),
        'config',
        'rubicks_search_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='yolo_detector',            
            executable='yolo_detector_node',    
            name='yolo_detector_node',          
            parameters=[config_path],
            output='screen'
        )
    ])