"""
Dog3DNav SLAM 建图 / 重定位 启动文件 (bringup 统一入口)

建图模式:
  ros2 launch bringup slam.launch.py mode:=mapping
  ros2 launch bringup slam.launch.py mode:=mapping rviz:=true

重定位模式(需要已有地图):
  ros2 launch bringup slam.launch.py mode:=relocation
  ros2 launch bringup slam.launch.py mode:=relocation init_pose:="[1.0,0.0,0.0,0.0,0.0,0.0]"

配置文件直接使用 super_lio 自带的 config/livox_360.yaml (建图)
和 config/relocation.yaml (重定位)，不维护 bringup 侧副本。

输出话题(已对导航栈做了适配 remap, 无需改动 navigation.launch.py):
  /odom            <- /lio/odom     (供 octo_planner / local_planner / pathFollower)
  /lidar_points    <- /lio/cloud_world  (供 latticePlanner 避障)
  /lio/imu/odom    <- 纯 IMU 预测里程计 (供调试)

TF 链:
  map -> livox_frame            (super_lio 动态发布)
  livox_frame -> base_link      (静态, 雷达在 base 前方, 俯角 15 deg)
  map -> odom                   (identity, 桥接导航栈)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('super_lio')

    # 直接使用 super_lio 自带配置
    mapping_config = os.path.join(pkg_slam, 'config', 'livox_360.yaml')
    reloc_config = os.path.join(pkg_slam, 'config', 'relocation.yaml')
    rviz_config = os.path.join(pkg_slam, 'rviz', 'lio.rviz')

    # ---- 公共参数 ----
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose = LaunchConfiguration('init_pose')

    ld = LaunchDescription([
        DeclareLaunchArgument('mode', default_value='relocation',
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
    # super_lio 发布 map->livox_frame (lio.global.imu_frame="livox_frame")。
    # 补齐 livox_frame->base_link (雷达在本体前方 0.10m, 下方 0.08m, 俯角 15 deg)
    # 以及 map->odom (identity)，确保 TF 链完整。
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
        arguments=['-0.10', '0', '-0.08', '0', '-0.261799', '0',
                   'livox_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # ---- SLAM 节点 ----
    # 通过 remap 把 lio 输出对齐导航栈输入:
    #   /odom         <- /lio/odom        (octo_planner + local_planner + pathFollower)
    #   /lidar_points <- /lio/cloud_world (latticePlanner 避障点云)
    slam_remaps = [
        ('/lio/odom', '/odom'),
        ('/lio/cloud_world', '/lidar_points'),
    ]

    # mapping 模式: 建图 (使用 livox_360.yaml)
    ld.add_action(Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[mapping_config,
                    {'use_sim_time': use_sim_time}],
        remappings=slam_remaps,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    ))

    # relocation 模式: 重定位 (使用 relocation.yaml, 需要已有地图)
    ld.add_action(Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[reloc_config,
                    {'use_sim_time': use_sim_time},
                    {'lio.relocation.init_pose': init_pose}],
        remappings=slam_remaps,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'relocation'"])),
    ))

    # ---- RViz2 ----
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    ))

    return ld
