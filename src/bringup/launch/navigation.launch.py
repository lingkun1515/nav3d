"""
Dog3DNav 完整导航栈启动文件

End-to-end flow:
  Web UI  /goal_pose  octo_planner  /planned_path  localPlanner (waypoint + TF)
          pathFollower  /cmd_vel (Twist)  Gazebo robot

用法:
  ros2 launch bringup navigation.launch.py pcd_file:=/path/to/map.pcd
  ros2 launch bringup navigation.launch.py  # 使用已有 from_pcd.world 或 empty.world
  ros2 launch bringup navigation.launch.py launch_rviz:=false  # 不启动 RViz2
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess, RegisterEventHandler,
                            OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_share = get_package_share_directory('simulation')
    local_share = get_package_share_directory('local_planner')
    bringup_share = get_package_share_directory('bringup')
    planner_share = get_package_share_directory('octo_planner')

    local_params = os.path.join(local_share, 'config', 'local_planner_params.yaml')
    planner_params = os.path.join(planner_share, 'config', 'planner_params.yaml')
    path_folder = os.path.join(local_share, 'paths')
    rviz_config = os.path.join(bringup_share, 'config', 'navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_file = LaunchConfiguration('pcd_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    empty_world = os.path.join(sim_share, 'worlds', 'empty.world')
    from_pcd_world = os.path.join(sim_share, 'worlds', 'from_pcd.world')
    pcd_to_world_script = os.path.join(sim_share, 'scripts', 'pcd_to_world.py')
    gazebo_launch_file = os.path.join(sim_share, 'launch', 'gazebo.launch.py')

    def _setup_gazebo(context):
        plc = context.perform_substitution(pcd_file)
        utc = context.perform_substitution(use_sim_time)

        if plc and not os.path.exists(from_pcd_world):
            gen = ExecuteProcess(
                cmd=[sys.executable, pcd_to_world_script, plc, from_pcd_world,
                     '--resolution', '0.3', '--max-boxes', '3000'],
                name='pcd_to_world',
                output='screen',
            )
            gz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={
                    'world': from_pcd_world,
                    'use_sim_time': utc,
                }.items(),
            )
            return [gen,
                    RegisterEventHandler(
                        OnProcessExit(target_action=gen, on_exit=[gz]))]

        world = from_pcd_world if os.path.exists(from_pcd_world) else empty_world
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': world, 'use_sim_time': utc}.items(),
        )]

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=from_pcd_world,
            description='Gazebo world file',
        ),
        DeclareLaunchArgument(
            'pcd_file', default_value='',
            description='PCD map file for octo_planner and Gazebo world generation',
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2 with the navigation config',
        ),
    ])

    # 1. Gazebo + robot
    ld.add_action(OpaqueFunction(function=_setup_gazebo))

    # 2. octo_planner
    ld.add_action(TimerAction(
        period=2.0,
        actions=[
            Node(
                package='octo_planner',
                executable='octo_planner_node',
                name='octo_planner_node',
                output='screen',
                parameters=[
                    planner_params,
                    {'pcd_file': pcd_file},
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ]
    ))

    # 3. localPlanner + pathFollower
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
                    {'global_frame_id': 'odom'},
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

    # 4. ROSBridge for Web UI
    ld.add_action(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    ))

    # 5. RViz2 (optional, controlled by launch_rviz)
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    ))

    return ld
