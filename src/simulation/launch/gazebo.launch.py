import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf.xacro')
    world_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # Local models directory (urban_2_story, urban_platform, etc.)
    # gitignored — download separately via: ign fuel download -v 4 -u <fuel_url>
    models_dir = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        model_path = f'{worlds_dir}:{models_dir}:{existing_model_path}'
    else:
        model_path = f'{worlds_dir}:{models_dir}'

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                pkg_share, 'worlds',
                'urban2_story.world'
                # 'empty.world'
                # 'map_nav3d.world'
            ),
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'x', default_value='-15.0', description='Robot initial X position'
        ),
        DeclareLaunchArgument(
            'y', default_value='-6.0', description='Robot initial Y position'
        ),
        DeclareLaunchArgument(
            'z', default_value='0.1', description='Robot initial Z position'
        ),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='Robot initial yaw'
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
                '-entity', 'diff_drive_robot',
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),

        # TF tree: diff_drive (WORLD mode) publishes map→base_footprint directly.
        # No static map→odom needed — odometry frame is now "map".
    ])
