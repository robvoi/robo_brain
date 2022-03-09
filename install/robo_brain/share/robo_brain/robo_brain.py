import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration(
        'serial_baudrate', default='115200')  # for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    publish_tf = LaunchConfiguration('publish_tf', default='true')

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'urdf_example'
    file_subpath = '/home/robo/robo_ws/robo_brain/urdf/example_robot.urdf.xacro'
    # Use xacro to process the file
    xacro_file = os.path.join(file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        DeclareLaunchArgument(
            'publish_tf',
            default_value=publish_tf,
            description='publish TD ODOM Base Link'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "laser"]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_link"]
        ),

        """
        Node(
           package='ros2_laser_scan_matcher',
           executable='laser_scan_matcher',
           parameters=[{'publish_odom':'odom',
                        'publish_tf': publish_tf,
                        'base_frame':'base_footprint'}]),

        """
        
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
                    'freq': 40.0}],
            arguments=['--ros-args', '--log-level', 'ERROR']
        ),


        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}],
            arguments=[file_subpath]),

    ])
