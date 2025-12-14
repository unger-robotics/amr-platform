#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose2D, TransformStamped, Quaternion # <--- Quaternion neu dazu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import math
import time

class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')

        # --- KONFIGURATION ---
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'

        # QoS Settings: MUSS zu ESP32 passen (Best Effort!)
        # Wenn hier "Reliable" steht und ESP32 "Best Effort" sendet,
        # kommen KEINE Daten an (Inkompatibilität).
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- SUBSCRIBER (Input vom ESP32) ---
        self.sub_raw = self.create_subscription(
            Pose2D,
            '/odom_raw',
            self.callback_odom_raw,
            qos_profile
        )

        # --- PUBLISHER (Output an Nav2/SLAM) ---
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- STATUS VARIABLEN (für Geschwindigkeitsberechnung) ---
        self.last_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.first_run = True

        self.get_logger().info('Odom Converter gestartet. Warte auf /odom_raw...')

    def callback_odom_raw(self, msg):
        current_time = self.get_clock().now()

        # 1. Quaternion berechnen (Yaw -> Quaternion)
        # ROS 2 nutzt Quaternionen für Rotationen, ESP32 sendet nur Theta (Euler)
        q = self.euler_to_quaternion(0, 0, msg.theta)

        # 2. TF Broadcast (odom -> base_link)
        # Dies ist zwingend für SLAM und RViz
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0 # 2D Roboter
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

        # 3. Odometry Nachricht erstellen
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # Position setzen
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Geschwindigkeit berechnen (Numerische Differentiation)
        # v = dx / dt
        if not self.first_run:
            dt_nanos = (current_time - self.last_time).nanoseconds
            dt = dt_nanos / 1e9

            if dt > 0:
                dx = msg.x - self.last_x
                dy = msg.y - self.last_y
                dtheta = msg.theta - self.last_theta

                # Normalize Angle Wrap (-Pi bis +Pi Sprung korrigieren)
                if dtheta > math.pi: dtheta -= 2 * math.pi
                if dtheta < -math.pi: dtheta += 2 * math.pi

                vx = dx / dt
                vy = dy / dt
                vth = dtheta / dt

                # Twist im Roboter-Frame (base_link) berechnen
                # Da Odometrie meist im odom-Frame ist, müssen wir es zurückrotieren
                # Für einfache Differential Drive Robots: vx ist lokale Vorwärtsgeschwindigkeit
                # Vereinfachung: Wir senden Twist im Odom Frame, Nav2 kann das handhaben
                odom.twist.twist.linear.x = math.sqrt(vx**2 + vy**2) # Annäherung
                odom.twist.twist.angular.z = vth

        # Kovarianzen setzen (Wichtig für EKF/Nav2!)
        # Kleine Werte = Hohes Vertrauen in die Daten
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 100.0, 0.0, 0.0, 0.0, # Z ignoriert
            0.0, 0.0, 0.0, 100.0, 0.0, 0.0, # Roll ignoriert
            0.0, 0.0, 0.0, 0.0, 100.0, 0.0, # Pitch ignoriert
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # Yaw
        ]

        self.pub_odom.publish(odom)

        # Update Status
        self.last_time = current_time
        self.last_x = msg.x
        self.last_y = msg.y
        self.last_theta = msg.theta
        self.first_run = False

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Konvertiert Euler-Winkel in Quaternion (x, y, z, w)
        """
        # Berechnung der Quaternion-Komponenten
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        # Erstellen des echten ROS 2 Objekts
        q_msg = Quaternion()
        q_msg.x = qx
        q_msg.y = qy
        q_msg.z = qz
        q_msg.w = qw

        return q_msg

def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
