"""
Full navigation stack launch:
  Gazebo sim + octo_planner + waypoint_follower + rosbridge

End-to-end flow:
  Web UI → /goal_pose → octo_planner → /planned_path
        → waypoint_follower → /cmd_vel → Gazebo robot
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_share = get_package_share_directory('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Try to find octo_planner share (may not be built yet)
    try:
        planner_share = get_package_share_directory('octo_planner')
        planner_params = os.path.join(planner_share, 'config', 'planner_params.yaml')
    except Exception:
        planner_params = ''

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_share, 'worlds', 'empty.world'),
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'pcd_file', default_value='',
            description='PCD map file for octo_planner'
        ),

        # Gazebo + Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # Waypoint Follower (simple path tracker)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='simulation',
                    executable='waypoint_follower.py',
                    name='waypoint_follower',
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        ),

        # ROSBridge for Web UI
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
