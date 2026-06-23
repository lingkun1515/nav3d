"""
Dog3DNav 导航栈启动文件

End-to-end flow:
  Web UI  /goal_pose  octo_planner  /planned_path  localPlanner  /path  pathFollower  /cmd_vel

用法:
  ros2 launch bringup navigation.launch.py
  ros2 launch bringup navigation.launch.py launch_rviz:=false
  ros2 launch bringup navigation.launch.py launch_rosbridge:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    local_share = get_package_share_directory('local_planner')
    bringup_share = get_package_share_directory('bringup')

    nav_params = os.path.join(bringup_share, 'config', 'navigation_config.yaml')
    path_folder = os.path.join(local_share, 'paths')
    rviz_config = os.path.join(bringup_share, 'config', 'navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_file = LaunchConfiguration('pcd_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rosbridge = LaunchConfiguration('launch_rosbridge')

    # Maps bundled with bringup package
    maps_dir = os.path.join(bringup_share, 'maps')
    map_bt = os.path.join(maps_dir, 'map_nav3d.bt')
    map_pcd = os.path.join(maps_dir, 'map_nav3d.pcd')
    default_map = map_bt if os.path.exists(map_bt) else (
        map_pcd if os.path.exists(map_pcd) else '')
    if default_map:
        default_map = os.path.realpath(default_map)  # resolve install symlink → src/

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'pcd_file', default_value=default_map,
            description='Map file (.bt/.pcd/.ot/.world/.sdf), default bringup/maps/map_nav3d.bt',
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2 with the navigation config',
        ),
        DeclareLaunchArgument('launch_rosbridge', default_value='false',
                              description='Launch rosbridge WebSocket for Web UI'),
    ])

    # 1. octo_planner
    ld.add_action(TimerAction(
        period=2.0,
        actions=[
            Node(
                package='octo_planner',
                executable='octo_planner_node',
                name='octo_planner_node',
                output='screen',
                parameters=[
                    nav_params,
                    {'pcd_file': pcd_file},
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ]
    ))

    # 2. Local planner (selectable via planner:=) + pathFollower
    ld.add_action(TimerAction(
        period=3.5,
        actions=[
            Node(
                package='local_planner',
                executable='latticePlanner',
                name='latticePlanner',
                output='screen',
                parameters=[
                    nav_params,
                    {'pathFolder': path_folder},
                    {'use_sim_time': use_sim_time},
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
                    nav_params,
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/state_estimation', '/odom'),
                ],
            ),
        ]
    ))

    # 3. ROSBridge for Web UI
    ld.add_action(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rosbridge),
    ))

    # 4. RViz2 (optional, controlled by launch_rviz)
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    ))

    return ld
