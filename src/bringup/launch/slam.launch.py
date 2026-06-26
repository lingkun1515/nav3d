"""
Dog3DNav SLAM 建图 / 重定位 启动文件

建图模式:
  ros2 launch bringup slam.launch.py
  ros2 launch bringup slam.launch.py mode:=mapping rviz:=true

重定位模式(需要已有地图):
  ros2 launch bringup slam.launch.py mode:=relocation init_pose:="[1.0,0.0,0.0,0.0,0.0,0.0]"

输出话题(已对导航栈做了适配 remap, 无需改动 navigation.launch.py):
  /odom            ← /lio/odom     (frame_id=map, 供 octo_planner / local_planner / pathFollower)
  /lidar_points    ← /lio/cloud_world  (frame_id=map, 供 latticePlanner 避障)
  /lio/imu/odom    ← 纯 IMU 预测里程计 (frame_id=map, 供调试)

TF 链:
  map → livox_frame            (super_lio 动态发布)
  livox_frame → base_link      (静态, 雷达在 base 前方 0.13m, 俯角 15°)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('super_lio')
    pkg_bringup = get_package_share_directory('bringup')

    # ---- 公共参数 ----
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose = LaunchConfiguration('init_pose')

    rviz_slam = os.path.join(pkg_slam, 'rviz', 'lio.rviz')
    rviz_reloc = os.path.join(pkg_slam, 'rviz', 'relocation.rviz')
    slam_config = os.path.join(pkg_bringup, 'config', 'slam_config.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description="SLAM 模式: 'mapping' (建图) 或 'relocation' (重定位)"),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='启动 RViz2 可视化'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用仿真时间'),
        DeclareLaunchArgument('init_pose',
                              default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
                              description='重定位初值 [x,y,z,roll,pitch,yaw]'),
    ])

    # ---- TF 桥接 ----
    # super_lio 发布 map→livox_frame (lio.global.imu_frame="livox_frame")。
    # 这里补齐 livox_frame→base_link (雷达在本体前方 0.15m, 上方 0.13m, 俯角 15°)
    # 以及 map→odom (identity)，确保 TF 链完整，RViz2 能渲染 base_link 帧的 /path
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_livox_to_base_link',
        arguments=['-0.10', '0', '-0.08', '0', '0.0', '0', 
                   'livox_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # ---- SLAM 节点 ----
    # 通过 remap 把 lio 输出对齐导航栈输入, 避免修改 navigation.launch.py:
    #   /odom         ← /lio/odom        (octo_planner + local_planner + pathFollower)
    #   /lidar_points ← /lio/cloud_world (latticePlanner 避障点云)
    slam_remaps = [
        ('/lio/odom', '/odom'),
        ('/lio/cloud_world', '/lidar_points'),
    ]

    # mapping 模式: 建图
    ld.add_action(Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[slam_config,
                    {'use_sim_time': use_sim_time}],
        remappings=slam_remaps,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    ))

    # relocation 模式: 重定位 (需要已有地图)
    ld.add_action(Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[slam_config,
                    {'use_sim_time': use_sim_time},
                    {'lio.relocation.init_pose': init_pose}],
        remappings=slam_remaps,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'relocation'"])),
    ))

    # ---- RViz2 (根据模式选不同 rviz 配置) ----
    # 注意: 多条件 IfCondition 用 LaunchConfiguration 组合。这里用两个独立 Node
    # 各自判断自己的条件即可。
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_slam, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time}],
        # 仅当 rviz:=true 且 mode:=mapping 时启动
        condition=IfCondition(rviz),
    ))
    # 重定位模式也用同一个 rviz 配置 (relocation.rviz)
    # 如果两种模式需要不同 rviz, 可以在此处切换

    return ld
