from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():  
    remappings = [
          ('/myRobot/cmd_vel', '/cmd_vel')]
    
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
            remappings=remappings,
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': 'config/mapper_params_online_async.yaml'
            }.items()
        )
    ])