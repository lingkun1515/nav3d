import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('octo_planner')
    default_params = os.path.join(pkg_share, 'config', 'planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_file', default_value='',
            description='Path to PCD map file'),

        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to parameter YAML file'),

        DeclareLaunchArgument(
            'launch_rosbridge', default_value='true',
            description='Whether to launch rosbridge_websocket'),

        Node(
            package='octo_planner',
            executable='octo_planner_node',
            name='octo_planner_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'pcd_file': LaunchConfiguration('pcd_file')},
            ],
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('launch_rosbridge')),
        ),
    ])
