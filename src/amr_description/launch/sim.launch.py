from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('amr_description'))
    robot_description = (package_share / 'urdf' / 'amr.urdf').read_text()
    gazebo_launch = Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo window'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={
                'world': str(package_share / 'worlds' / 'demo.world'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'amr', '-topic', 'robot_description', '-z', '0.02'],
            output='screen',
        ),
    ])
