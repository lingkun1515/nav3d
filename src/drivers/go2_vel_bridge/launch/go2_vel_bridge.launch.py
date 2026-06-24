import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_vel_bridge')
    default_config = os.path.join(pkg_share, 'config', 'go2_vel_bridge.yaml')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Parameter file for go2_vel_bridge',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        Node(
            package='go2_vel_bridge',
            executable='go2_vel_bridge_node',
            name='go2_vel_bridge',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
