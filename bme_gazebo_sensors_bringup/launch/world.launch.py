import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_runtime = get_package_share_directory('bme_gazebo_sensors')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my.sdf',
        description='World file name'
    )

    world_path = PathJoinSubstitution([
        pkg_runtime,
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo = ExecuteProcess(
        cmd=['ruby', '/usr/bin/ign', 'gazebo', world_path, '-r', '-v', '-v1', '--force-version', '6'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo
    ])