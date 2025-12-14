from setuptools import setup

package_name = 'amr_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan',
    maintainer_email='jan@todo.todo',
    description='Bridge Node for ESP32 Odometry',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_converter = amr_bridge.odom_converter:main',
        ],
    },
)
