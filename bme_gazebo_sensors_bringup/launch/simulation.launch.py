import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_sensors = get_package_share_directory('bme_gazebo_sensors')
    pkg_bringup = get_package_share_directory('bme_gazebo_sensors_bringup')

    world_name = LaunchConfiguration('world').perform(context)
    model_name = LaunchConfiguration('model').perform(context)
    x_value = LaunchConfiguration('x').perform(context)
    y_value = LaunchConfiguration('y').perform(context)
    yaw_value = LaunchConfiguration('yaw').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    world_path = os.path.join(pkg_sensors, 'worlds', world_name)
    model_path = os.path.join(pkg_sensors, 'urdf', model_name)
    expanded_model_path = '/tmp/mogi_bot.expanded.urdf'

    robot_description_xml = xacro.process_file(model_path).toxml()

    with open(expanded_model_path, 'w', encoding='utf-8') as expanded_file:
        expanded_file.write(robot_description_xml)

    gz_resource_paths = os.pathsep.join([
        os.path.join(pkg_sensors, 'worlds'),
        os.path.join(pkg_sensors, 'meshes'),
        os.path.join(pkg_sensors, 'urdf'),
        os.path.join(pkg_sensors, '..'),
    ])

    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-s', '-r', world_path],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='3',
    )

    gz_websocket = ExecuteProcess(
        cmd=[
            'gz', 'launch', '-v', '4',
            os.path.join(pkg_bringup, 'config', 'gazebo_websocket.gzlaunch'),
        ],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='3',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time.lower() == 'true',
            'robot_description': ParameterValue(robot_description_xml, value_type=str),
        }],
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
                    '--world', 'empty',
                    '--file', expanded_model_path,
                    '--name', 'mogi_bot',
                    '-x', x_value,
                    '-y', y_value,
                    '-z', '0.1',
                    '-Y', yaw_value,
                ],
            )
        ],
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='3',
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_paths),
        gz_sim_server,
        gz_websocket,
        robot_state_publisher_node,
        gz_bridge,
        spawn_robot,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Open RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='rviz.rviz',
            description='RViz config file name inside the bringup rviz directory',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='my.sdf',
            description='World file name inside bme_gazebo_sensors/worlds',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='mogi_bot.urdf',
            description='URDF/Xacro file name inside bme_gazebo_sensors/urdf',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='2.5',
            description='Spawn x',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='1.5',
            description='Spawn y',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='-1.5707',
            description='Spawn yaw',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        OpaqueFunction(function=launch_setup),
    ])