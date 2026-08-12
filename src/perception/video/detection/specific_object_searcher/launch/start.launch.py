from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'network_interface',
            default_value='enx00e04c36003f',
            description='Сетевой интерфейс для подключения к роботу'
        ),
        DeclareLaunchArgument(
            'target_class',
            default_value='red ball',
            description='Целевой класс для поиска'
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.5',
            description='Порог уверенности детекции'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Базовый фрейм робота'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Фрейм одометрии'
        ),
        
        Node(
            package='specific_object_searcher',
            executable='specific_object_searcher_node',
            name='specific_object_searcher_node',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'target_class': LaunchConfiguration('target_class'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'video_timeout': 3,
            }],
            output='screen'
        ),
    ])