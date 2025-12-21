"""
amr.launch.py - Startet alle AMR-Komponenten fuer Phase 4

Startet:
  - RPLidar A1 (sllidar_ros2)
  - robot_state_publisher (URDF -> statische TFs)
  - odom_converter (/odom_raw -> /odom + TF)

TF-Baum nach Start:
  odom -> base_footprint -> base_link -> laser

Aufruf:
  ros2 launch amr_bringup amr.launch.py
  ros2 launch amr_bringup amr.launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch-Argumente
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )
    serial_port = LaunchConfiguration('serial_port')

    # Package-Pfade
    pkg_description = get_package_share_directory('amr_description')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    
    # URDF einlesen
    urdf_file = os.path.join(pkg_description, 'urdf', 'amr.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        serial_port_arg,
        
        # === RPLidar A1 ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
            ),
            launch_arguments={'serial_port': serial_port}.items()
        ),
        
        # === Robot State Publisher (statische TFs aus URDF) ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }]
        ),
        
        # === Odom Converter (dynamische TF: odom -> base_footprint) ===
        Node(
            package='amr_bridge',
            executable='odom_converter',
            name='odom_converter',
            output='screen'
        ),
    ])
