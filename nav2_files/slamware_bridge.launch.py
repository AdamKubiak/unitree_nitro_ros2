"""
Launch file for slamware_bridge — connects to Slamtec Mapper and publishes
/scan, /map, /mapper/pose topics and map→base_link TF.

Usage:
  ros2 launch slamware_bridge slamware_bridge.launch.py
  ros2 launch slamware_bridge slamware_bridge.launch.py mapper_ip:=192.168.11.1 scan_rate:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---- Arguments ----
        DeclareLaunchArgument('mapper_ip', default_value='192.168.11.1',
            description='IP address of the Slamtec Mapper'),
        DeclareLaunchArgument('mapper_port', default_value='1445',
            description='TCP port of the Mapper'),
        DeclareLaunchArgument('scan_rate', default_value='10.0',
            description='Laser scan publish rate (Hz)'),
        DeclareLaunchArgument('map_rate', default_value='0.5',
            description='Occupancy grid publish rate (Hz)'),
        DeclareLaunchArgument('scan_frame_id', default_value='laser_frame',
            description='Frame ID for laser scan'),
        DeclareLaunchArgument('map_frame_id', default_value='map',
            description='Frame ID for map'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link',
            description='Frame ID for robot base'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom',
            description='Frame ID for odometry (used as child of map TF)'),

        # ---- Node ----
        Node(
            package='slamware_bridge',
            executable='slamware_bridge_node',
            name='slamware_bridge',
            output='screen',
            parameters=[{
                'mapper_ip': LaunchConfiguration('mapper_ip'),
                'mapper_port': LaunchConfiguration('mapper_port'),
                'scan_rate': LaunchConfiguration('scan_rate'),
                'map_rate': LaunchConfiguration('map_rate'),
                'scan_frame_id': LaunchConfiguration('scan_frame_id'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            }],
        ),
    ])
