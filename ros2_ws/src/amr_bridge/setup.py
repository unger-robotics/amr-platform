from setuptools import setup

package_name = 'amr_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Unger',
    maintainer_email='jan@unger.de',
    description='AMR Bridge Nodes - Odom Converter',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_converter = amr_bridge.odom_converter:main',
        ],
    },
)
