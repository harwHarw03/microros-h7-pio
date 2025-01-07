from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_visualize'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools', 'numpy', 'transforms3d'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='A package to visualize IMU data and odometry from a Micro-ROS device.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_odometry_node = imu_visualize.imu_odometry_node:main',
            'imu_visualizer = imu_visualize.imu_visualizer_node:main',
            'pg_euler_plot = imu_visualize.pg_euler_plot:main',
        ],
    },
)
