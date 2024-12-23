from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    # Parametri configurabili
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Dichiarazione degli argomenti
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([fra2mo_dir, 'maps', 'mappa_mondo_1.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    # Include per la navigazione AMCL
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Nodo ArUco
    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_detect',
        parameters=[
            {
                'image_is_rectified': True,
                'marker_id': 115,
                'marker_size': 0.2,
                'reference_frame': 'camera_link',
                'camera_frame': 'camera_link',
                'marker_frame': 'aruco_marker_frame',
            }
        ],
        remappings=[
            ('/image', '/videocamera'),
            ('/camera_info', '/videocamera_info'),
        ],
        output='screen',
    )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Restituzione della descrizione del lancio
    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            nav2_bringup_launch,
            aruco_single,  # Nodo ArUco
            rviz_node,  # Nodo RViz2
        ]
    )

