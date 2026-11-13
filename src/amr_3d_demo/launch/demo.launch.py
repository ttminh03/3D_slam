from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('point_topic', default_value='/points'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        Node(
            package='amr_3d_demo',
            executable='demo_node',
            name='amr_3d_demo',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'point_topic': LaunchConfiguration('point_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }],
        ),
    ])
