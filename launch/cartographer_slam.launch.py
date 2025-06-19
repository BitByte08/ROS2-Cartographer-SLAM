import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Cartographer 설정 파일 경로
    cartographer_config_dir = PathJoinSubstitution([
        get_package_share_directory('ros2_lidar_cartographer'),
        'config'
    ]).perform(context)

    configuration_basename = LaunchConfiguration('configuration_basename').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # 센서 토픽 이름
    scan_topic = LaunchConfiguration('scan_topic').perform(context) # LiDAR 스캔 토픽

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('scan', scan_topic), # 여러분의 LiDAR 스캔 토픽에 맞게 변경하세요! (예: '/laser_scan')
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'resolution': resolution}],
        remappings=[
            ('map', 'map'), # 지도 토픽 이름 (RViz에서 구독)
            ('scan_matched_points2', 'scan_matched_points2') # Cartographer에서 발행하는 정합된 포인트 클라우드 토픽
        ]
    )

    # --- URDF 없이 static_transform_publisher로 TF 변환 발행 ---
    # base_link에서 laser_frame으로의 고정 변환 (x y z roll pitch yaw)
    # 이 값을 여러분의 로봇에 맞게 정확히 설정해야 합니다.
    # 예시: base_link 앞 0.1m, 높이 0.1m에 LiDAR가 장착된 경우
    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        output='screen',
        arguments=[
            '0.1', '0', '0.1',  # x y z (미터)
            '0', '0', '0',      # roll pitch yaw (라디안)
            'base_link',        # 부모 프레임 (parent frame)
            'laser_frame'       # 자식 프레임 (child frame)
        ]
    )
    # --------------------------------------------------------

    return [
        cartographer_node,
        occupancy_grid_node,
        static_tf_publisher_node # static_transform_publisher 노드 추가
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='ydlidar_x2_config.lua',
            description='Name of cartographer configuration file to load.'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of the map.'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan', # LiDAR 스캔 토픽 (실제 로봇 토픽으로 변경 필요!)
            description='LiDAR scan topic.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        OpaqueFunction(function=launch_setup)
    ])