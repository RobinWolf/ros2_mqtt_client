from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'ros2mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files (same as Cmakes InstallDirectory)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include configs
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Include nodes executables
        (os.path.join('share', package_name, package_name), glob('ros2mqtt_bridge*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rw39401',
    maintainer_email='robin.wolf@fft.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_client_publisher_node = ros2mqtt_bridge.mqtt_client_publisher_node:main',
        ],
    },
)
