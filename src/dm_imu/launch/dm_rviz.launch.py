from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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
        ),

        # RVIZ2节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('dm_imu'),
                'rviz',
                'imu_tf.rviz'
            ])],
            output='screen'
        )
    ])