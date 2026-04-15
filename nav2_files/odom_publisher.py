#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_unitree_legged_msgs.msg import HighState

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(HighState, 'high_state', self.state_cb, 10)


    def state_cb(self, msg):
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])
        t.transform.rotation.w = float(msg.imu.quaternion[0])
        t.transform.rotation.x = float(msg.imu.quaternion[1])
        t.transform.rotation.y = float(msg.imu.quaternion[2])
        t.transform.rotation.z = float(msg.imu.quaternion[3])
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])

        odom.pose.pose.orientation.w = float(msg.imu.quaternion[0])
        odom.pose.pose.orientation.x = float(msg.imu.quaternion[1])
        odom.pose.pose.orientation.y = float(msg.imu.quaternion[2])
        odom.pose.pose.orientation.z = float(msg.imu.quaternion[3])

        # Pose covariance diagonal: x, y, z, roll, pitch, yaw
        odom.pose.covariance[0]  = 0.01    # x
        odom.pose.covariance[7]  = 0.01    # y
        odom.pose.covariance[14] = 0.001   # z
        odom.pose.covariance[21] = 0.001   # roll
        odom.pose.covariance[28] = 0.001   # pitch
        odom.pose.covariance[35] = 0.05    # yaw

        # Velocity from Go1 high_state
        odom.twist.twist.linear.x  = float(msg.velocity[0])
        odom.twist.twist.linear.y  = float(msg.velocity[1])
        odom.twist.twist.angular.z = float(msg.yaw_speed)

        # Twist covariance diagonal: vx, vy, vz, wx, wy, wz
        odom.twist.covariance[0]  = 0.01    # vx
        odom.twist.covariance[7]  = 0.01    # vy
        odom.twist.covariance[14] = 0.001   # vz
        odom.twist.covariance[21] = 0.001   # wx
        odom.twist.covariance[28] = 0.001   # wy
        odom.twist.covariance[35] = 0.05    # wz

        self.odom_pub.publish(odom)




def main():
    rclpy.init()
    rclpy.spin(OdomPublisher())
    print("ey")
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()