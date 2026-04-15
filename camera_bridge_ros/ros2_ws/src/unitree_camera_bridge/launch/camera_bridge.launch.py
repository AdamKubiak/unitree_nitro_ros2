from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'jetson_ip', default_value='192.168.123.13',
            description='IP address of the Jetson Nano running the stream'),
        DeclareLaunchArgument(
            'port', default_value='8080',
            description='Port of the MJPEG stream server'),
        DeclareLaunchArgument(
            'camera_name', default_value='head_front',
            description='Camera name for topic namespace'),
        DeclareLaunchArgument(
            'frame_id', default_value='camera_head_front',
            description='TF frame ID for the camera'),

        Node(
            package='unitree_camera_bridge',
            executable='camera_bridge',
            name='camera_bridge',
            output='screen',
            parameters=[{
                'stream_url': [
                    'http://',
                    LaunchConfiguration('jetson_ip'),
                    ':',
                    LaunchConfiguration('port'),
                    '/stream'
                ],
                'camera_name': LaunchConfiguration('camera_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_compressed': True,
                'reconnect_interval': 3.0,
            }]
        ),
    ])
