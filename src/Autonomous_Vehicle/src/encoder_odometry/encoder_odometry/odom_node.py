#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class EncoderOdometry(Node):

    def __init__(self):
        super().__init__('encoder_odometry')

        # Robot geometry (tune these during odometry calibration):
        #   ros2 run encoder_odometry odom_node --ros-args -p wheel_base:=0.30
        self.WHEEL_RADIUS = self.declare_parameter(
            'wheel_radius', 0.04).value
        self.WHEEL_BASE = self.declare_parameter(
            'wheel_base', 0.26).value
        self.CPR = self.declare_parameter(
            'counts_per_rev', 4684.0).value
        self.publish_tf = self.declare_parameter(
            'publish_tf', True).value

        # Zero the velocity if encoder data stops arriving.
        self.ENCODER_TIMEOUT = 0.25

        self.subscription = self.create_subscription(
            Int64MultiArray,
            '/encoder_counts',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.timer = self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info(
            f"Encoder Odometry Started (r={self.WHEEL_RADIUS}, "
            f"base={self.WHEEL_BASE}, cpr={self.CPR})"
        )

    def encoder_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning("Received fewer than two encoder counts")
            return

        left = msg.data[0]
        right = msg.data[1]
        now = self.get_clock().now()

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_time = now
            return

        d_left = left - self.prev_left
        d_right = right - self.prev_right
        dt = (now - self.prev_time).nanoseconds * 1e-9

        self.prev_left = left
        self.prev_right = right
        self.prev_time = now

        meters_per_count = (2.0 * math.pi * self.WHEEL_RADIUS) / self.CPR

        left_distance = d_left * meters_per_count
        right_distance = d_right * meters_per_count

        d = (left_distance + right_distance) / 2.0
        d_theta = (right_distance - left_distance) / self.WHEEL_BASE

        self.x += d * math.cos(self.theta + d_theta / 2.0)
        self.y += d * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        if dt > 1e-3:
            self.linear_velocity = d / dt
            self.angular_velocity = d_theta / dt

    def publish_odometry(self):
        now = self.get_clock().now()

        if (self.prev_time is None or
                (now - self.prev_time).nanoseconds * 1e-9 >
                self.ENCODER_TIMEOUT):
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        qx, qy, qz, qw = quaternion_from_euler(
            0.0, 0.0, self.theta
        )

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        # Diagonal covariance: x, y, z, roll, pitch, yaw.
        # Unused axes get a large value for a planar robot.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 1e6
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
