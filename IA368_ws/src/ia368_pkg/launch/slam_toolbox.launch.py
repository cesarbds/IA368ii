from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ia368_pkg',
            executable='lidar_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='tf_node_slam_toolbox',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='vel_node',
            output='screen'
        )
    ])