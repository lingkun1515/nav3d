import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
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
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # Local models directory
    models_dir = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        model_path = f'{worlds_dir}:{models_dir}:{existing_model_path}'
    else:
        model_path = f'{worlds_dir}:{models_dir}'

    # URDF selection: car or a1
    urdf_file_car = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    urdf_file_a1 = os.path.join(pkg_share, 'urdf', 'a1', 'a1.urdf.xacro')
    meshes_dir = os.path.join(pkg_share, 'urdf', 'meshes')
    os.environ['A1_MESHES_DIR'] = meshes_dir

    # Use PythonExpression to pick URDF at launch time
    urdf_expr = PythonExpression([
        '"', urdf_file_car, '" if "', robot_model, '" == "car" else "', urdf_file_a1, '"'
    ])

    robot_description = ParameterValue(Command(['xacro ', urdf_expr]), value_type=str)

    # Plugin paths: our custom plugin + standard gazebo_ros + system default
    plugin_path = os.path.join(pkg_share, '..', '..', 'lib')
    ros_gazebo_plugins = '/opt/ros/humble/lib'
    system_gazebo_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if existing_plugin_path:
        gazebo_plugin_path = f'{plugin_path}:{ros_gazebo_plugins}:{system_gazebo_plugins}:{existing_plugin_path}'
    else:
        gazebo_plugin_path = f'{plugin_path}:{ros_gazebo_plugins}:{system_gazebo_plugins}'

    # Set os.environ directly so gzserver.launch.py can read them.
    # gzserver.launch.py's generate_launch_description() reads os.environ
    # (not the launch context), so SetEnvironmentVariable alone does not
    # propagate to the inner launch file's Python code.
    os.environ['GAZEBO_MODEL_PATH'] = model_path
    os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path

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
    entity_name = PythonExpression([
        '"diff_drive_robot" if "', robot_model, '" == "car" else "a1"'
    ])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 
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

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
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
