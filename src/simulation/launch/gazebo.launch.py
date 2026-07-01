import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    slam_mode = LaunchConfiguration('slam_mode')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # Local models directory + system Gazebo models (sun etc.)
    models_dir = os.path.join(pkg_share, 'models')
    hospital_models_dir = os.path.join(models_dir, 'hospital')
    system_models = '/usr/share/gazebo-11/models'
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        model_path = f'{worlds_dir}:{models_dir}:{hospital_models_dir}:{system_models}:{existing_model_path}'
    else:
        model_path = f'{worlds_dir}:{models_dir}:{hospital_models_dir}:{system_models}'

    # URDF selection: car or a1
    urdf_file_car = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    urdf_file_a1 = os.path.join(pkg_share, 'urdf', 'a1', 'a1.urdf.xacro')
    meshes_dir = os.path.join(pkg_share, 'urdf', 'meshes')
    os.environ['A1_MESHES_DIR'] = meshes_dir

    # Use PythonExpression to pick URDF at launch time
    urdf_expr = PythonExpression([
        '"', urdf_file_car, '" if "', robot_model, '" == "car" else "', urdf_file_a1, '"'
    ])

    # In SLAM mode, suppress A1RLController's odom + TF (SLAM provides both).
    # xacro args are only consumed by a1 URDF; car URDF ignores them silently.
    publish_odom_val = PythonExpression([
        '"false" if "', slam_mode, '" == "true" else "true"'
    ])
    publish_tf_val = PythonExpression([
        '"false" if "', slam_mode, '" == "true" else "true"'
    ])

    robot_description = ParameterValue(Command([
        'xacro ', urdf_expr,
        ' publish_odom:=', publish_odom_val,
        ' publish_tf:=', publish_tf_val,
    ]), value_type=str)

    # Plugin path: our custom plugin dir (A1RLController.so) + stock gazebo_ros
    # + system defaults. The simulation package.xml also exports
    # <gazebo_ros plugin_path="${prefix}/lib"> so GazeboRosPaths discovers it
    # too — this prevents a leading-colon in the merged GAZEBO_PLUGIN_PATH that
    # breaks libgazebo_ros_factory.so service registration on Foxy.
    plugin_path = os.path.join(pkg_share, '..', '..', 'lib')
    ros_gazebo_plugins = os.path.join('/opt', 'ros', os.environ.get('ROS_DISTRO', 'foxy'), 'lib')
    system_gazebo_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    parts = [plugin_path, ros_gazebo_plugins, system_gazebo_plugins]
    if existing_plugin_path:
        parts.append(existing_plugin_path)
    gazebo_plugin_path = ':'.join(parts)

    os.environ['GAZEBO_MODEL_PATH'] = model_path
    # Note: do NOT set os.environ['GAZEBO_PLUGIN_PATH'] here. gzserver.launch.py
    # merges it with GazeboRosPaths output; the package.xml export handles
    # discovery. We propagate the full path via SetEnvironmentVariable below.

    # LD_LIBRARY_PATH for ONNX Runtime libs — installed to lib/simulation/
    onnx_lib_dir = os.path.join(pkg_share, '..', '..', 'lib', 'simulation')
    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if existing_ld_path:
        ld_library_path = f'{onnx_lib_dir}:{existing_ld_path}'
    else:
        ld_library_path = onnx_lib_dir
    os.environ['LD_LIBRARY_PATH'] = ld_library_path

    # Spawn Z and entity name per model
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = PythonExpression([
        '"0.1" if "', robot_model, '" == "car" else "0.3"'
    ])
    spawn_yaw = LaunchConfiguration('yaw')
    launch_rosbridge = LaunchConfiguration('launch_rosbridge')
    gui = LaunchConfiguration('gui')
    entity_name = PythonExpression([
        '"diff_drive_robot" if "', robot_model, '" == "car" else "a1"'
    ])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
        # Disable online model database fetch — prevents gzserver from blocking
        # on http://models.gazebosim.org during startup (network-restricted envs).
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds',
                                    #    'hospital_two_floors_stripped.world'
                                    #    'hospital_two_floors.world'   # needs gazebo model downloads
                                       'urban2_story.world'
                                    #    'empty_world.world'
                                    #    'map_nav3d.world'
                                       ),
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='a1',
            description='Robot model: car or a1'
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='false',
            description='SLAM mode: suppress onboard odom/TF so SLAM provides them.'
                        ' launch slam.launch.py separately.'
        ),
        DeclareLaunchArgument(
            'x', default_value='0.0', description='Robot initial X position'
        ),
        DeclareLaunchArgument(
            'y', default_value='-0.0', description='Robot initial Y position'
        ),
        DeclareLaunchArgument(
            'z', default_value='0.1',
            description='Robot initial Z (car: 0.1, a1: auto-set to 0.3)'
        ),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='Robot initial yaw'
        ),
        DeclareLaunchArgument(
            'launch_rosbridge', default_value='true',
            description='Launch rosbridge WebSocket for Web UI'
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Set to "false" to run Gazebo headless (no gzclient)'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file, 'gui': gui}.items(),
        ),

        # Robot State Publisher (URDF static TFs: base_link → trunk → legs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # Non-SLAM mode: static TFs to bridge simulation → navigation
        #   map → odom (identity): connects global frame to odometry frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(slam_mode),
        ),
        #   base_link → livox_frame: sensor frame (LiDAR at x=+0.15, z=+0.13, pitch=-15°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox_tf',
            arguments=['0.15', '0', '0.13', '0', '-0.261799', '0',
                       'base_link', 'livox_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(slam_mode),
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity_name,
                '-topic', 'robot_description',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        ),

        # ROSBridge WebSocket (for Web UI)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(launch_rosbridge),
        ),
    ])
