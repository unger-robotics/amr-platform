"""
Launch-File für AMR Serial Bridge
Startet die Serial-Bridge mit konfigurierbaren Parametern
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch-Argumente deklarieren
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial connection'
    )
    
    # Serial Bridge Node
    serial_bridge_node = Node(
        package='amr_serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'send_rate': 20.0,
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        serial_bridge_node,
    ])
