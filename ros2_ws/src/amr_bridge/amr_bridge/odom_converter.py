#!/usr/bin/env python3
"""
odom_converter.py - Konvertiert /odom_raw (Pose2D) zu /odom (Odometry) + TF

Publiziert:
  - /odom (nav_msgs/Odometry)
  - TF: odom -> base_footprint
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')

        # QoS Profil fuer micro-ROS Kompatibilitaet (Best Effort)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber mit Best Effort QoS (wie ESP32 Publisher)
        self.subscription = self.create_subscription(
            Pose2D,
            '/odom_raw',
            self.odom_callback,
            qos_best_effort
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Vorherige Werte fuer Velocity-Berechnung
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.prev_time = self.get_clock().now()
        self.first_msg = True

        self.get_logger().info('odom_converter started')
        self.get_logger().info('  Subscribing: /odom_raw (Pose2D) [Best Effort QoS]')
        self.get_logger().info('  Publishing:  /odom (Odometry)')
        self.get_logger().info('  Publishing:  TF odom -> base_footprint')

    def odom_callback(self, msg: Pose2D):
        current_time = self.get_clock().now()

        # Erstes Mal: nur Werte speichern
        if self.first_msg:
            self.prev_x = msg.x
            self.prev_y = msg.y
            self.prev_theta = msg.theta
            self.prev_time = current_time
            self.first_msg = False
            return

        # Zeit-Delta berechnen
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.001:
            dt = 0.05  # Fallback: 20 Hz

        # Velocity berechnen (Differenz im Weltkoordinaten)
        dx = msg.x - self.prev_x
        dy = msg.y - self.prev_y
        dtheta = self._normalize_angle(msg.theta - self.prev_theta)

        # Velocity im Body-Frame
        cos_theta = math.cos(msg.theta)
        sin_theta = math.sin(msg.theta)
        vx_body = (dx * cos_theta + dy * sin_theta) / dt
        vy_body = (-dx * sin_theta + dy * cos_theta) / dt
        vtheta = dtheta / dt

        # Odometry Message erstellen
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        # Orientation (Quaternion aus theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)

        # Velocity (im Body-Frame)
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = vtheta

        # Covariance (Diagonalelemente)
        # Pose Covariance (6x6: x, y, z, roll, pitch, yaw)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[14] = 1e6   # z (nicht verwendet)
        odom.pose.covariance[21] = 1e6   # roll (nicht verwendet)
        odom.pose.covariance[28] = 1e6   # pitch (nicht verwendet)
        odom.pose.covariance[35] = 0.03  # yaw

        # Twist Covariance
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[7] = 0.01   # vy
        odom.twist.covariance[35] = 0.03  # vtheta

        self.odom_pub.publish(odom)

        # TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        # Werte speichern
        self.prev_x = msg.x
        self.prev_y = msg.y
        self.prev_theta = msg.theta
        self.prev_time = current_time

    def _normalize_angle(self, angle):
        """Normalisiert Winkel auf [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
