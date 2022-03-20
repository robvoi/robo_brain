import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import xacro


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    # bringup package
    bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {}
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    # bringup end

    return LaunchDescription([
        # bringup start
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'params_file',
            default_value='/home/robo/robo_ws/robo_brain/nav2_config/nav2_params.yaml',
            description='Full path to the ROS2 parameters file to use'),

        # bringup end

        DeclareLaunchArgument(
            'slam_params_file',
            default_value='/home/robo/robo_ws/robo_brain/nav2_config/mapper_params_online_async.yaml',
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "-b", "230400", "-D", "/dev/ttyUSB1"]
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True, }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                   'laser_scan_topic': '/scan',
                   'odom_topic': '/odom',
                   'publish_tf': True,
                   'base_frame_id': 'base_footprint',
                   'odom_frame_id': 'odom',
                   'init_pose_from_topic': '',
                   'freq': 6.0}],
            arguments=['--ros-args', '--log-level', 'ERROR']
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro.process_file(
                '/home/robo/robo_ws/robo_brain/urdf/robot.urdf.xacro').toxml()}],
            arguments=['/home/robo/robo_ws/robo_brain/urdf/robot.urdf.xacro']),

        Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': False}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
        # bringup start
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
        # bringup end

    ])
