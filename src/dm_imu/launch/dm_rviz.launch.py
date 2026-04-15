from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 dm_imu 包的共享目录路径
    dm_imu_share_dir = get_package_share_directory('dm_imu')
    config_file = os.path.join(dm_imu_share_dir, 'config', 'imu_config.yaml')

    return LaunchDescription([
        # IMU节点
        Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 921600
            }, config_file]
        ),
        
        # 静态TF：map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id', 'map',
                '--child-frame-id', 'base_link'
            ],
            output='screen'
        ),

        # RVIZ2节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('dm_imu'),
                'rviz',
                'imu_data.rviz'
            ])],
            output='screen'
        )
    ])