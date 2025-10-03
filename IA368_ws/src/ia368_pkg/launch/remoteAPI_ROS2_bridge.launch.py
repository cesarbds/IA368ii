from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ia368_pkg',
            namespace='autodocking',
            executable='battery_node',
            name='battery_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='autodocking',
            executable='bumper_and_velocity_node',
            name='bumper_and_velocity_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='autodocking',
            executable='charging_base_node',
            name='charging_base_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='autodocking',
            executable='docking_node',
            name='docking_node',
            output='screen'
        )
    ])
