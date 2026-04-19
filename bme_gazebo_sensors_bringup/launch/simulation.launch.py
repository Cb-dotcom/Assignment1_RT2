import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_sensors = get_package_share_directory('bme_gazebo_sensors')
    pkg_bringup = get_package_share_directory('bme_gazebo_sensors_bringup')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='rviz.rviz',
        description='RViz config file name inside the bringup rviz directory'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my.sdf',
        description='World file name inside bme_gazebo_sensors/worlds'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='mogi_bot.urdf',
        description='URDF file name inside bme_gazebo_sensors/urdf'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='2.5',
        description='Spawn x'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='1.5',
        description='Spawn y'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='-1.5707',
        description='Spawn yaw'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    world_path = PathJoinSubstitution([pkg_sensors, 'worlds', LaunchConfiguration('world')])
    model_path = PathJoinSubstitution([pkg_sensors, 'urdf', LaunchConfiguration('model')])

    gz_resource_paths = os.pathsep.join([
        os.path.join(pkg_sensors, 'worlds'),
        os.path.join(pkg_sensors, 'meshes'),
        os.path.join(pkg_sensors, 'urdf'),
        os.path.join(pkg_sensors, '..'),
    ])

    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-s', '-r', world_path],
        output='screen'
    )

    gz_websocket = ExecuteProcess(
        cmd=[
            'gz', 'launch', '-v', '4',
            os.path.join(pkg_bringup, 'config', 'gazebo_websocket.gzlaunch')
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': open(
                    os.path.join(pkg_sensors, 'urdf', 'mogi_bot.urdf'),
                    'r',
                    encoding='utf-8'
                ).read()
            }
        ]
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_mogi_bot',
                output='screen',
                arguments=[
                    '-world', 'empty',
                    '-file', model_path,
                    '-name', 'mogi_bot',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', '0.1',
                    '-Y', LaunchConfiguration('yaw'),
                ],
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_config_arg,
        world_arg,
        model_arg,
        x_arg,
        y_arg,
        yaw_arg,
        sim_time_arg,

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_paths),

        gz_sim_server,
        gz_websocket,
        robot_state_publisher_node,
        spawn_robot,
    ])