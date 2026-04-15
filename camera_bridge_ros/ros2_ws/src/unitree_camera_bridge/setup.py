from setuptools import setup
import os
from glob import glob

package_name = 'unitree_camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Kubiak',
    maintainer_email='adam@todo.com',
    description='Bridge MJPEG camera stream from Unitree Go1 Jetson to ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_bridge = unitree_camera_bridge.camera_bridge_node:main',
        ],
    },
)
