from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 dm_imu 包的共享目录路径
    dm_imu_share_dir = get_package_share_directory('dm_imu')
    config_file = os.path.join(dm_imu_share_dir, 'config', 'imu_config.yaml')
    
    return LaunchDescription([
        Node(
            package='dm_imu',       
            executable='dm_imu_node',  # 可执行文件名
            name='dm_imu_node',     # 节点名称（可自定义）
            output='screen',        # 输出到屏幕
            parameters=[{           # 参数配置
                'port': '/dev/ttyACM0',
                'baud': 921600
            }, config_file],
            arguments=['--ros-args', '--log-level', 'INFO']
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
        )
    ])
