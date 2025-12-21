"""
description.launch.py - Startet robot_state_publisher und odom_converter

Startet:
  - robot_state_publisher: Publiziert statische TFs aus URDF
  - odom_converter: Konvertiert /odom_raw -> /odom + TF odom->base_footprint

TF-Baum nach Start:
  odom -> base_footprint -> base_link -> laser
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # URDF-Datei finden
    pkg_description = get_package_share_directory('amr_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'amr.urdf')
    
    # URDF einlesen
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot State Publisher (statische TFs aus URDF)
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
        
        # Odom Converter (dynamische TF: odom -> base_footprint)
        Node(
            package='amr_bridge',
            executable='odom_converter',
            name='odom_converter',
            output='screen'
        ),
    ])
