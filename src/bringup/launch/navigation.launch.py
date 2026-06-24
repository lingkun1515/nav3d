"""
Dog3DNav 导航栈启动文件

End-to-end flow:
  Web UI  /goal_pose  octo_planner  /planned_path  localPlanner  /path  pathFollower  /cmd_vel
  (optional) go2_vel_bridge  /cmd_vel → unitree_api /sport_request

用法:
  ros2 launch bringup navigation.launch.py
  ros2 launch bringup navigation.launch.py launch_rviz:=false
  ros2 launch bringup navigation.launch.py launch_rosbridge:=false
  ros2 launch bringup navigation.launch.py launch_vel_bridge:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    local_share = get_package_share_directory('local_planner')
    bringup_share = get_package_share_directory('bringup')
    vel_bridge_share = get_package_share_directory('go2_vel_bridge')

    nav_params = os.path.join(bringup_share, 'config', 'navigation_config.yaml')
    path_folder = os.path.join(local_share, 'paths')
    rviz_config = os.path.join(bringup_share, 'config', 'navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_file = LaunchConfiguration('pcd_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rosbridge = LaunchConfiguration('launch_rosbridge')
    launch_vel_bridge = LaunchConfiguration('launch_vel_bridge')

    # Maps bundled with bringup package
    maps_dir = os.path.join(bringup_share, 'maps')
    map_bt = os.path.join(maps_dir, 'map_nav3d.bt')
    map_pcd = os.path.join(maps_dir, 'map_nav3d.pcd')
    default_map = map_bt if os.path.exists(map_bt) else (
        map_pcd if os.path.exists(map_pcd) else '')
    if default_map:
        default_map = os.path.realpath(default_map)  # resolve install symlink → src/

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
        DeclareLaunchArgument('launch_vel_bridge', default_value='false',
                              description='Launch go2_vel_bridge to forward cmd_vel to Unitree Go2'),
    ])

    # 1. octo_planner
    ld.add_action(Node(
        package='octo_planner',
        executable='octo_planner_node',
        name='octo_planner_node',
        output='screen',
        parameters=[
            nav_params,
            {'pcd_file': pcd_file},
            {'use_sim_time': use_sim_time},
        ],
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

    # 4. go2_vel_bridge (optional, controlled by launch_vel_bridge)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vel_bridge_share, 'launch', 'go2_vel_bridge.launch.py')
        ),
        launch_arguments=[('use_sim_time', use_sim_time)],
        condition=IfCondition(launch_vel_bridge),
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
