from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/path/to/map.yaml',
            description='Full path to map yaml file'),
        
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time if true'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'min_particles': 500,
                'max_particles': 2000,
                'kld_err': 0.01,
                'kld_z': 0.99,
                'odom_model_type': 'diff',
                'update_min_d': 0.25,
                'update_min_a': 0.2,
                'resample_interval': 1,
                'transform_tolerance': 1.0,
                'recovery_alpha_slow': 0.0,
                'recovery_alpha_fast': 0.0,
                'base_frame_id': 'base_link'
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )

    ])
