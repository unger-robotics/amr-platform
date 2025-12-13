#!/usr/bin/env python3
"""
AMR Serial Bridge mit Odometrie (Phase 2)

Subscribes: /cmd_vel (geometry_msgs/Twist)
Publishes:  /odom (nav_msgs/Odometry)
Broadcasts: TF odom → base_link

Serial-Protokoll:
    Host → ESP32:  V:<m/s>,W:<rad/s>\n
    ESP32 → Host:  ODOM:<left>,<right>,<x>,<y>,<theta>\n

Version: 0.4.0-odom
Date: 2025-12-12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial
import threading
import math


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Parameter
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Serial-Verbindung
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            raise

        # Subscriber für /cmd_vel
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        # Publisher für /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometrie-Zustand (für Geschwindigkeitsberechnung)
        self.last_odom_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        # Serial-Lesethread
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('Serial Bridge with Odometry ready')

    def cmd_vel_callback(self, msg: Twist):
        """Sendet Geschwindigkeitsbefehl an ESP32"""
        v = msg.linear.x
        w = msg.angular.z

        cmd = f"V:{v:.3f},W:{w:.3f}\n"
        try:
            self.serial.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def serial_read_loop(self):
        """Liest Serial-Daten in separatem Thread"""
        buffer = ""

        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Zeilen verarbeiten
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.process_serial_line(line)

            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')

    def process_serial_line(self, line: str):
        """Verarbeitet eine Serial-Zeile"""
        # ODOM:<left>,<right>,<x>,<y>,<theta>
        if line.startswith('ODOM:'):
            try:
                parts = line[5:].split(',')
                if len(parts) >= 5:
                    ticks_left = int(parts[0])
                    ticks_right = int(parts[1])
                    x = float(parts[2])
                    y = float(parts[3])
                    theta = float(parts[4])

                    self.publish_odometry(x, y, theta, ticks_left, ticks_right)

            except (ValueError, IndexError) as e:
                self.get_logger().debug(f'ODOM parse error: {e}')

        # OK-Bestätigung loggen
        elif line.startswith('OK:'):
            self.get_logger().debug(f'ESP32: {line}')

        # Fehler und Warnungen
        elif line.startswith('ERR:') or line.startswith('FAILSAFE:'):
            self.get_logger().warn(f'ESP32: {line}')

        # Startup-Nachricht
        elif 'ready' in line.lower():
            self.get_logger().info(f'ESP32: {line}')

    def publish_odometry(self, x: float, y: float, theta: float,
                         ticks_left: int, ticks_right: int):
        """Publiziert Odometrie und TF"""
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9

        # Geschwindigkeiten aus Position ableiten (falls dt > 0)
        if dt > 0.001:
            vx = (x - self.last_x) / dt
            vy = (y - self.last_y) / dt

            # Winkelgeschwindigkeit mit Wraparound
            dtheta = theta - self.last_theta
            if dtheta > math.pi:
                dtheta -= 2 * math.pi
            elif dtheta < -math.pi:
                dtheta += 2 * math.pi
            vtheta = dtheta / dt
        else:
            vx = vy = vtheta = 0.0

        # Werte speichern
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_odom_time = now

        # --- Odometry Message ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Quaternion aus Theta (Rotation um Z-Achse)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Kovarianz (diagonal, grobe Schätzung)
        # [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        # Twist (Geschwindigkeiten im base_link Frame)
        # Transformation von Welt → Roboter
        odom.twist.twist.linear.x = vx * math.cos(theta) + vy * math.sin(theta)
        odom.twist.twist.linear.y = -vx * math.sin(theta) + vy * math.cos(theta)
        odom.twist.twist.angular.z = vtheta

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

        # --- TF Broadcast ---
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(theta / 2.0)
        tf.transform.rotation.w = math.cos(theta / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        """Aufräumen beim Beenden"""
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SerialBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
