"""
Dog3DNav 完整导航栈启动文件

End-to-end flow:
  Web UI  /goal_pose  octo_planner  /planned_path  localPlanner (waypoint + TF)
          pathFollower  /cmd_vel (Twist)  Gazebo robot

用法:
  ros2 launch bringup navigation.launch.py
  ros2 launch bringup navigation.launch.py pcd_file:=/path/to/map.pcd
  ros2 launch bringup navigation.launch.py launch_rviz:=false

Gazebo 场景需离线生成（两种方式）：
  # 方式1: PCD → world
  python3 src/simulation/scripts/pcd_to_world.py <map.pcd> <output.world>
  # 方式2: BT → world
  python3 src/simulation/scripts/bt_to_world.py <map.bt> <output.world>
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
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

    # Maps bundled with bringup package
    maps_dir = os.path.join(bringup_share, 'maps')
    map_bt = os.path.join(maps_dir, 'map_nav3d.bt')
    map_pcd = os.path.join(maps_dir, 'map_nav3d.pcd')
    default_map = map_bt if os.path.exists(map_bt) else (
        map_pcd if os.path.exists(map_pcd) else '')
    if default_map:
        default_map = os.path.realpath(default_map)  # resolve install symlink → src/

    empty_world = os.path.join(sim_share, 'worlds', 'empty.world')
    nav3d_world = os.path.join(sim_share, 'worlds', 'map_nav3d.world')
    path_world = nav3d_world if os.path.exists(nav3d_world) else empty_world
    gazebo_launch_file = os.path.join(sim_share, 'launch', 'gazebo.launch.py')

    x_arg = LaunchConfiguration('x')
    y_arg = LaunchConfiguration('y')
    z_arg = LaunchConfiguration('z')
    yaw_arg = LaunchConfiguration('yaw')
    launch_sim = LaunchConfiguration('launch_sim')

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=path_world,
            description='Gazebo world file (generate offline: pcd_to_world.py / bt_to_world.py)',
        ),
        DeclareLaunchArgument(
            'pcd_file', default_value=default_map,
            description='Map file (.bt/.pcd/.ot/.world/.sdf), default bringup/maps/map_nav3d.bt',
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2 with the navigation config',
        ),
        DeclareLaunchArgument(
            'launch_sim', default_value='false',
            description='Launch Gazebo simulation (set false for real robot)',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot initial X (m)'),
        DeclareLaunchArgument('y', default_value='-6.0', description='Robot initial Y (m)'),
        DeclareLaunchArgument('z', default_value='0.1', description='Robot initial Z (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot initial yaw (rad)'),
    ])

    # 1. Gazebo + robot (optional, controlled by launch_sim)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world': path_world,
            'use_sim_time': use_sim_time,
            'x': x_arg,
            'y': y_arg,
            'z': z_arg,
            'yaw': yaw_arg,
        }.items(),
        condition=IfCondition(launch_sim),
    ))

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
                    {'use_laser_scan': False},
                    {'use_planned_path': True},
                    {'global_frame_id': 'map'},
                    {'corridor_trust_mode': True},
                ],
                remappings=[
                    ('/state_estimation', '/odom'),
                    ('/registered_scan', '/lidar_points'),
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
