from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    message_arg = DeclareLaunchArgument(
        'message',
        default_value='Hello, Sunrise',
        description='Message to publish'
    )

    example_node = Node(
        package='example_package_ros',
        executable='example_node',
        name='example_node',
        output='screen',
        parameters=[{'message': LaunchConfiguration('message')}]
    )

    return LaunchDescription([
        message_arg,
        example_node,
    ])
