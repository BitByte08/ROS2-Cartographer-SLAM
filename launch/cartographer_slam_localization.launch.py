from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 런치 아규먼트 선언
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=LaunchConfiguration('config_dir'),
            description='Full path to config file directory'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='ydlidar_x2_config_localization.lua',
            description='Name of lua configuration file'),
        
        DeclareLaunchArgument(
            'load_state_filename',
            default_value=LaunchConfiguration('pbstream_file'),
            description='Path to pbstream file'),

        # Cartographer Node 실행 (localization-only)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                '-configuration_directory', LaunchConfiguration('configuration_directory'),
                '-configuration_basename', LaunchConfiguration('configuration_basename'),
                '-load_state_filename', LaunchConfiguration('load_state_filename'),
                '-load_frozen_state', 'true'  # SLAM 꺼짐 → localization 전용
            ]
        ),

        # Occupancy Grid 생성기 (선택)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }]
        )
    ])
