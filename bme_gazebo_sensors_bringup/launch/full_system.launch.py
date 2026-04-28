import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bme_gazebo_sensors_bringup')
    pkg_sensors = get_package_share_directory('bme_gazebo_sensors')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='rviz.rviz', description='RViz config file')
    world_arg = DeclareLaunchArgument('world', default_value='my.sdf', description='Name of the Gazebo world file to load')
    model_arg = DeclareLaunchArgument('model', default_value='mogi_bot.urdf', description='Name of the URDF description to load')
    x_arg = DeclareLaunchArgument('x', default_value='2.5', description='x coordinate of spawned robot')
    y_arg = DeclareLaunchArgument('y', default_value='1.5', description='y coordinate of spawned robot')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='-1.5707', description='yaw angle of spawned robot')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Flag to enable use_sim_time')
    start_rosbridge_arg = DeclareLaunchArgument('start_rosbridge', default_value='true', description='Launch rosbridge websocket server')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    motion_executor_config = PathJoinSubstitution([pkg_bringup, 'config', 'motion_executor.yaml'])
    goal_bridge_config = PathJoinSubstitution([pkg_bringup, 'config', 'goal_bridge.yaml'])
    rviz_config_path = PathJoinSubstitution([pkg_sensors, 'rviz', LaunchConfiguration('rviz_config')])

    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
        parameters=[
            {'port': 9090},
            {'address': '0.0.0.0'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    component_container = ComposableNodeContainer(
        name='motion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        composable_node_descriptions=[
            ComposableNode(
                package='bme_gazebo_sensors',
                plugin='bme_gazebo_sensors::MotionExecutorComponent',
                name='motion_executor',
                parameters=[
                    motion_executor_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            ),
            ComposableNode(
                package='bme_gazebo_sensors',
                plugin='bme_gazebo_sensors::GoalBridgeComponent',
                name='goal_bridge',
                parameters=[
                    goal_bridge_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            ),
        ],
        output='screen'
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
        start_rosbridge_arg,
        simulation_launch,
        rosbridge_websocket_node,
        rosapi_node,
        rviz_node,
        map_to_odom_tf,
        component_container,
    ])