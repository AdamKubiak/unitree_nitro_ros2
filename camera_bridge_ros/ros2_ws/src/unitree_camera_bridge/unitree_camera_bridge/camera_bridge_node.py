#!/usr/bin/env python3
"""
ROS 2 Camera Bridge Node for Unitree Go1.
Connects to the MJPEG stream from the Jetson and publishes frames
as standard sensor_msgs/Image topics for use in RViz, Nav2, etc.

Usage:
    ros2 run unitree_camera_bridge camera_bridge --ros-args \
        -p stream_url:="http://192.168.123.13:8080/stream" \
        -p camera_name:="head_front"

    Or use the launch file:
    ros2 launch unitree_camera_bridge camera_bridge.launch.py
"""

import cv2
import numpy as np
import threading
from urllib.request import urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class CameraBridgeNode(Node):
    """Bridges an MJPEG HTTP stream to ROS 2 Image topics."""

    def __init__(self):
        super().__init__('camera_bridge')

        # Declare parameters
        self.declare_parameter('stream_url',
                               'http://192.168.123.13:8080/stream')
        self.declare_parameter('camera_name', 'head_front')
        self.declare_parameter('frame_id', 'camera_head_front')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('reconnect_interval', 3.0)

        # Get parameters
        self.stream_url = self.get_parameter('stream_url') \
            .get_parameter_value().string_value
        self.camera_name = self.get_parameter('camera_name') \
            .get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id') \
            .get_parameter_value().string_value
        self.publish_compressed = self.get_parameter('publish_compressed') \
            .get_parameter_value().bool_value
        self.reconnect_interval = self.get_parameter('reconnect_interval') \
            .get_parameter_value().double_value

        # QoS for image transport (best effort for camera streams)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        topic_base = f'/camera/{self.camera_name}'
        self.image_pub = self.create_publisher(
            Image, f'{topic_base}/image_raw', image_qos)

        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage,
                f'{topic_base}/image_raw/compressed',
                image_qos)

        self.info_pub = self.create_publisher(
            CameraInfo, f'{topic_base}/camera_info', image_qos)

        # Shared state between reader thread and publish timer
        self._latest_frame = None
        self._latest_jpeg = None
        self._frame_lock = threading.Lock()
        self.frame_count = 0

        # Background thread reads the MJPEG stream continuously
        self._reader_thread = threading.Thread(
            target=self._stream_reader_loop, daemon=True)
        self._reader_thread.start()

        # Timer publishes the latest frame at ~30 Hz
        self.create_timer(0.033, self._publish_callback)

        self.get_logger().info(
            f'Camera bridge starting: {self.stream_url} -> {topic_base}/')

    def _stream_reader_loop(self):
        """Background thread: connects to MJPEG stream and reads frames."""
        import time

        while rclpy.ok():
            # Connect
            stream = None
            try:
                self.get_logger().info(
                    f'Connecting to stream: {self.stream_url}')
                stream = urlopen(self.stream_url, timeout=5)
                self.get_logger().info('Connected to camera stream!')
            except (URLError, OSError, Exception) as e:
                self.get_logger().warn(
                    f'Cannot connect: {e}. Retrying in '
                    f'{self.reconnect_interval}s...')
                time.sleep(self.reconnect_interval)
                continue

            # Read frames continuously with large buffer reads
            buf = b''
            try:
                while rclpy.ok():
                    chunk = stream.read(65536)
                    if not chunk:
                        break

                    buf += chunk

                    # Extract all complete JPEG frames from buffer
                    while True:
                        start = buf.find(b'\xff\xd8')
                        end = buf.find(b'\xff\xd9')
                        if start == -1 or end == -1 or end <= start:
                            break

                        jpeg_bytes = buf[start:end + 2]
                        buf = buf[end + 2:]

                        np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            with self._frame_lock:
                                self._latest_frame = frame
                                self._latest_jpeg = jpeg_bytes

                    # Prevent buffer from growing unbounded
                    if len(buf) > 500000:
                        last_start = buf.rfind(b'\xff\xd8')
                        if last_start > 0:
                            buf = buf[last_start:]
                        else:
                            buf = buf[-10000:]

            except (OSError, Exception) as e:
                self.get_logger().warn(f'Stream error: {e}. Reconnecting...')

            if stream:
                try:
                    stream.close()
                except Exception:
                    pass
            time.sleep(1.0)

    def _publish_callback(self):
        """Timer callback: publishes the latest frame."""
        with self._frame_lock:
            frame = self._latest_frame
            jpeg_bytes = self._latest_jpeg

        if frame is None:
            return

        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        # Publish raw Image
        img_msg = Image()
        img_msg.header = header
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()
        self.image_pub.publish(img_msg)

        # Publish compressed
        if self.publish_compressed and jpeg_bytes is not None:
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = 'jpeg'
            comp_msg.data = jpeg_bytes
            self.compressed_pub.publish(comp_msg)

        # Publish CameraInfo
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.height = frame.shape[0]
        info_msg.width = frame.shape[1]
        self.info_pub.publish(info_msg)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Published {self.frame_count} frames '
                f'({frame.shape[1]}x{frame.shape[0]})')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
