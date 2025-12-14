from setuptools import setup
import os
from glob import glob

package_name = 'amr_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Falls du Launch-Files in diesem Paket hättest (hast du aktuell aber in amr_bringup)
        # (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan',
    maintainer_email='jan@todo.todo',
    description='Bridge for ESP32 Odometry',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Linke Seite: Name des Befehls (ros2 run amr_bridge <NAME>)
            # Rechte Seite: Pfad zur Funktion (Ordner.Datei:Funktion)
            'odom_converter = amr_bridge.odom_converter:main',
        ],
    },
)
