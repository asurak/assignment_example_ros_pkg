from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    example_node = Node(
        package='example_package_ros',
        executable='example_node',
        name='example_node'
    )

    advanced_example_node = Node(
        package='example_package_ros',
        executable='advanced_example_node',
        name='advanced_example_node'
    )

    return LaunchDescription([
        example_node,
        advanced_example_node,
    ])
