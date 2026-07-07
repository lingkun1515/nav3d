"""
Dog3DNav SLAM 建图 / 重定位 启动文件 (bringup 统一入口)

建图模式:
  ros2 launch bringup slam.launch.py mode:=mapping
  ros2 launch bringup slam.launch.py mode:=mapping rviz:=true

重定位模式(需要已有地图):
  ros2 launch bringup slam.launch.py mode:=relocation
  ros2 launch bringup slam.launch.py mode:=relocation init_pose:="[1.0,0.0,0.0,0.0,0.0,0.0]"

重定位 + global_reloc 全局重定位 (super_lio 不靠配置 init_pose, 而是订阅
global_reloc 估计的 /initial_pose 作为初始预估):
  # 1) 先用 global_reloc 的 build_map_cli 从 super_lio 的先验地图生成 .gkey
  #    (地图必须与 super_lio 加载的同一份 .pcd):
  #    build_map_cli --pcd src/slam/src/super_lio/map/map.pcd \
  #                  --out /tmp/reloc_map --name map --voxel 0.4
  # 2) 启动:
  ros2 launch bringup slam.launch.py mode:=relocation global_reloc:=true \
      map_key_path:=/tmp/reloc_map/map.gkey

配置文件: bringup/config/slam_config.yaml (建图+重定位合并, 两模式共用)

输出话题(已对导航栈做了适配 remap, 无需改动 navigation.launch.py):
  /odom            <- /lio/odom     (供 octo_planner / local_planner / pathFollower)
  /lidar_points    <- /lio/cloud_world  (供 latticePlanner 避障)
  /lio/imu/odom    <- 纯 IMU 预测里程计 (供调试)

global_reloc 模式额外话题:
  /initial_pose        <- global_reloc 发布 (PoseWithCovarianceStamped, frame=map)
  /reloc_confidence    <- global_reloc 单次置信度 (诊断用)
  /reloc_reliable      <- global_reloc 时序一致性可靠标志
  super_lio relocation_node 订阅 /initial_pose (use_external_init_pose=true),
  在收到可靠位姿前不初始化 (等待 global_reloc), 收到后用其作为 NDT/ICP 初值。

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
    pkg_bringup = get_package_share_directory('bringup')
    pkg_reloc = get_package_share_directory('global_reloc')

    slam_config = os.path.join(pkg_bringup, 'config', 'slam_config.yaml')
    reloc_params = os.path.join(pkg_reloc, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_slam, 'rviz', 'lio.rviz')

    # ---- 公共参数 ----
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose = LaunchConfiguration('init_pose')
    global_reloc = LaunchConfiguration('global_reloc')
    map_key_path = LaunchConfiguration('map_key_path')
    reloc_gate_publish = LaunchConfiguration('reloc_gate_publish')
    reloc_strategy = LaunchConfiguration('reloc_strategy')

    ld = LaunchDescription([
        DeclareLaunchArgument('mode', default_value='relocation',
                              description="SLAM 模式: 'mapping' (建图) 或 'relocation' (重定位)"),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='启动 RViz2 可视化'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用仿真时间'),
        DeclareLaunchArgument('init_pose',
                              default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
                              description='重定位初值 [x,y,z,roll,pitch,yaw] (global_reloc:=true 时被忽略)'),
        DeclareLaunchArgument('global_reloc', default_value='false',
                              description="true=重定位时启用 global_reloc, super_lio 订阅 /initial_pose 作为初始预估 "
                                          "(需配合 map_key_path); false=用配置 init_pose (默认, 行为不变)"),
        DeclareLaunchArgument('map_key_path', default_value='',
                              description='global_reloc 的 .gkey 地图索引 (global_reloc:=true 时必填, '
                                          '须与 super_lio 先验 .pcd 同源; 用 build_map_cli 生成)'),
        DeclareLaunchArgument('reloc_gate_publish', default_value='true',
                              description='global_reloc 是否仅在时序一致性可靠时才发布 /initial_pose '
                                          '(true=等可靠位姿再让 super_lio 初始化, 推荐)'),
        DeclareLaunchArgument('reloc_strategy', default_value='fast',
                              description="global_reloc 粗匹配策略: 'fast' (~0.7s, BEV+法向yaw+GICP, "
                                          "在线推荐—super_lio 自带 ICP 会精配), 'ndt_gicp' (~90s, 暴力 NDT "
                                          "网格, 最高独立精度), 'bev' (~15s)"),
    ])

    # ---- TF 桥接 ----
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
    slam_remaps = [
        ('/lio/odom', '/odom'),
        ('/lio/cloud_world', '/lidar_points'),
    ]

    # global_reloc=true 时, super_lio 改用外部 init_pose (等待 /initial_pose);
    # 否则用配置 init_pose (默认 /initialpose 订阅, 行为不变)。
    use_external_init_pose = PythonExpression(
        ["'true' if '", global_reloc, "' == 'true' else 'false'"])
    init_pose_topic = PythonExpression(
        ["'/initial_pose' if '", global_reloc, "' == 'true' else '/initialpose'"])

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

    # relocation 模式: 重定位 (需要已有地图, init_pose 由 launch 参数覆盖;
    # global_reloc=true 时改用外部 init_pose)
    ld.add_action(Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[slam_config,
                    {'use_sim_time': use_sim_time},
                    {'lio.relocation.init_pose': init_pose},
                    {'lio.relocation.use_external_init_pose': use_external_init_pose},
                    {'lio.relocation.init_pose_topic': init_pose_topic}],
        remappings=slam_remaps,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'relocation'"])),
    ))

    # ---- global_reloc 节点 (仅 global_reloc=true 且 mode=relocation) ----
    reloc_cond = PythonExpression(
        ["'", global_reloc, "' == 'true' and '", mode, "' == 'relocation'"])
    ld.add_action(Node(
        package='global_reloc',
        executable='reloc_node',
        name='global_reloc_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'lidar_topic': '/livox/lidar'},
            {'imu_topic': '/livox/imu'},
            {'init_pose_topic': '/initial_pose'},
            {'map_key_path': map_key_path},
            {'params_path': reloc_params},
            {'gate_publish': reloc_gate_publish},
            {'coarse_strategy': reloc_strategy},
        ],
        condition=IfCondition(reloc_cond),
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
