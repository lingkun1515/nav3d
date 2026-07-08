"""
Dog3DNav 无避障导航启动文件（跳过 latticePlanner）

数据流:
  Web UI  /goal_pose  octo_planner  /planned_path  pathFollower  /cmd_vel
                                  (use_global_path=true, TF map→base_link)

用法:
  ros2 launch bringup navigation_no_avoidance.launch.py
  ros2 launch bringup navigation_no_avoidance.launch.py launch_rviz:=false
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
    bringup_share = get_package_share_directory('bringup')

    nav_params = os.path.join(bringup_share, 'config', 'navigation_config.yaml')
    rviz_config = os.path.join(bringup_share, 'config', 'navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_file = LaunchConfiguration('pcd_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rosbridge = LaunchConfiguration('launch_rosbridge')

    # Maps bundled with bringup package
    maps_dir = os.path.join(bringup_share, 'maps')
    map_bt = os.path.join(maps_dir, 'topsun.bt')
    map_pcd = os.path.join(maps_dir, 'topsun.pcd')
    default_map = map_bt if os.path.exists(map_bt) else (
        map_pcd if os.path.exists(map_pcd) else '')
    if default_map:
        default_map = os.path.realpath(default_map)

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'pcd_file', default_value=default_map,
            description='Map file (.bt/.pcd/.ot/.world/.sdf)',
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument('launch_rosbridge', default_value='false',
                              description='Launch rosbridge WebSocket'),
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

    # 2. pathFollower in global-path mode (NO latticePlanner)
    ld.add_action(TimerAction(
        period=3.5,
        actions=[
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

    # 4. go2_vel_bridge (auto)
    try:
        gvb_share = get_package_share_directory('go2_vel_bridge')
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gvb_share, 'launch', 'go2_vel_bridge.launch.py')
            ),
            launch_arguments=[('use_sim_time', use_sim_time)],
        ))
    except Exception:
        pass

    # 5. RViz2
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    ))

    return ld
