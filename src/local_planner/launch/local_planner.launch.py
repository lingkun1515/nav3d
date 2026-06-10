import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('local_planner')
    default_params = os.path.join(pkg_share, 'config', 'local_planner_params.yaml')
    default_paths = os.path.join(pkg_share, 'paths')

    return LaunchDescription([
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[
                default_params,
                {'pathFolder': default_paths},
            ],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[default_params],
        ),
    ])
