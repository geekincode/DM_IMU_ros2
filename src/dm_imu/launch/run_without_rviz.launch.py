from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dm_imu',       
            executable='dm_imu_node',  # 可执行文件名
            name='dm_imu_node',     # 节点名称（可自定义）
            output='screen',        # 输出到屏幕
            parameters=[{           # 参数配置
                'port': '/dev/ttyACM0',
                'baud': 921600
            }]
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
