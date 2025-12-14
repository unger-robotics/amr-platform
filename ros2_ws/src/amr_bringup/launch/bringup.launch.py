import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # --- PFADE DEFINIEREN ---
    pkg_bringup = get_package_share_directory('amr_bringup')
    pkg_description = get_package_share_directory('amr_description')

    # Pfad zur URDF/XACRO Datei
    # HINWEIS: Wir nehmen an, die Datei heißt amr.urdf.xacro. Bitte Namen prüfen!
    xacro_file = os.path.join(pkg_description, 'urdf', 'amr.urdf.xacro')

    # --- NODES DEFINIEREN ---

    # 1. Robot State Publisher (Verarbeitet URDF und veröffentlicht statische TFs)
    # Nutzt 'xacro', um das Modell zur Laufzeit zu parsen
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': False
        }]
    )

    # 2. Joint State Publisher (Fake - nötig solange ESP32 keine Joint-States sendet)
    # Verhindert Fehlermeldungen in RViz über fehlende Transformationen der Räder
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 3. AMR Bridge (Der Odom Converter)
    # Wandelt micro-ROS Pose2D in Standard Odometrie um
    bridge_node = Node(
        package='amr_bridge',
        executable='odom_converter',
        name='odom_converter',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # 4. (Optional) RPLIDAR Treiber
    # Hier vorbereitet, aber auskommentiert für Phase 3
    # rplidar_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',
    #         'frame_id': 'laser_frame',
    #         'angle_compensate': True,
    #         'scan_mode': 'Standard'
    #     }]
    # )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        bridge_node,
        # rplidar_node
    ])
