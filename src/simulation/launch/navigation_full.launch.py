"""
Full navigation stack with local_planner (C++ algorithm nodes).

This is the same as navigation.launch.py — the "full" pipeline is now the default.
Kept for backwards compatibility.

End-to-end flow:
  Web UI  /goal_pose  octo_planner  /planned_path  localPlanner (waypoint mgmt + TF)
          pathFollower  /cmd_vel (Twist)  Gazebo robot
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess, RegisterEventHandler,
                            OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_share = get_package_share_directory('simulation')
    local_share = get_package_share_directory('local_planner')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    local_params = os.path.join(local_share, 'config', 'local_planner_params.yaml')
    path_folder = os.path.join(local_share, 'paths')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_file = LaunchConfiguration('pcd_file')

    empty_world = os.path.join(sim_share, 'worlds', 'empty.world')
    auto_world = '/tmp/dog3dnav_auto.world'
    pcd_to_world_script = os.path.join(sim_share, 'scripts', 'pcd_to_world.py')
    gazebo_launch_file = os.path.join(sim_share, 'launch', 'gazebo.launch.py')

    try:
        planner_share = get_package_share_directory('octo_planner')
        planner_launch = os.path.join(planner_share, 'launch', 'planner.launch.py')
        has_planner = True
    except Exception:
        has_planner = False

    def _setup_gazebo(context):
        plc = context.perform_substitution(pcd_file)
        utc = context.perform_substitution(use_sim_time)

        if plc and os.path.exists(plc):
            gen = ExecuteProcess(
                cmd=[sys.executable, pcd_to_world_script, plc, auto_world,
                     '--resolution', '0.2'],
                name='pcd_to_world',
                output='screen',
            )
            gz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={
                    'world': auto_world,
                    'use_sim_time': utc,
                }.items(),
            )
            return [gen,
                    RegisterEventHandler(
                        OnProcessExit(target_action=gen, on_exit=[gz]))]
        else:
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={
                    'world': empty_world,
                    'use_sim_time': utc,
                }.items(),
            )]

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=empty_world,
            description='Gazebo world file',
        ),
        DeclareLaunchArgument(
            'pcd_file', default_value='',
            description='PCD map file for octo_planner and Gazebo world generation',
        ),
    ])

    ld.add_action(OpaqueFunction(function=_setup_gazebo))

    if has_planner:
        ld.add_action(TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(planner_launch),
                    launch_arguments={
                        'pcd_file': pcd_file,
                        'launch_rosbridge': 'false',
                    }.items(),
                ),
            ]
        ))

    ld.add_action(TimerAction(
        period=3.5,
        actions=[
            Node(
                package='local_planner',
                executable='localPlanner',
                name='localPlanner',
                output='screen',
                parameters=[
                    local_params,
                    {'pathFolder': path_folder},
                    {'autonomyMode': True},
                    {'autonomySpeed': 0.5},
                    {'maxSpeed': 0.5},
                    {'use_sim_time': use_sim_time},
                    {'use_laser_scan': True},
                    {'use_planned_path': True},
                ],
                remappings=[
                    ('/state_estimation', '/odom'),
                ],
            ),
            Node(
                package='local_planner',
                executable='pathFollower',
                name='pathFollower',
                output='screen',
                parameters=[
                    local_params,
                    {'autonomyMode': True},
                    {'autonomySpeed': 0.5},
                    {'maxSpeed': 0.5},
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/state_estimation', '/odom'),
                ],
            ),
        ]
    ))

    ld.add_action(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    ))

    return ld
