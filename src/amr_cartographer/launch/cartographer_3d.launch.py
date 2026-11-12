import os
from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Keep the staged configuration alive for the lifetime of this launch process.
_config_directory = None


def generate_launch_description():
    global _config_directory
    _config_directory = TemporaryDirectory(prefix='amr_cartographer_')
    config_dir = Path(_config_directory.name)
    prefixes = [Path(get_package_prefix('cartographer_ros'))]
    prefixes.extend(Path(path) for path in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if path)
    core_config_dir = next(
        (prefix / 'share' / 'cartographer' / 'configuration_files' for prefix in prefixes
         if (prefix / 'share' / 'cartographer' / 'configuration_files' / 'map_builder.lua').is_file()),
        None,
    )
    amr_config = Path(get_package_share_directory('amr_cartographer')) / 'config' / 'amr_3d.lua'
    if core_config_dir is None:
        raise FileNotFoundError('Cartographer core Lua defaults missing from ROS prefixes')

    # Cartographer resolves Lua includes in its configuration directory. Stage
    # our config alongside the upstream defaults without modifying either package.
    for upstream_lua in core_config_dir.glob('*.lua'):
        (config_dir / upstream_lua.name).symlink_to(upstream_lua)
    (config_dir / amr_config.name).symlink_to(amr_config)

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-configuration_directory', str(config_dir),
                '-configuration_basename', 'amr_3d.lua',
            ],
            remappings=[('points2', '/points'), ('imu', '/imu/data')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
