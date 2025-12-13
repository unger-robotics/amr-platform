from setuptools import setup

package_name = 'amr_serial_bridge'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Unger',
    maintainer_email='unger.robotics@gmail.com',
    description='Serial bridge between ROS 2 cmd_vel and ESP32',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = amr_serial_bridge.serial_bridge:main',
        ],
    },
)
