# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     return LaunchDescription([
#         Node(
#             package='imu_dummy_visualizer',
#             executable='imu_publisher',
#             name='imu_publisher_node',
#             output='screen'
#         ),

#     ])

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('imu_visualize'),
        'config',
        'default.rviz'
    )
    return LaunchDescription([        
        Node(
            package='imu_visualize',
            executable='imu_odometry_node',
            name='imu_odometry_node',
            output='screen',
            parameters=[{'footstep_distance': 0.4}]
        ),
        Node(
            package='imu_visualize',
            executable='imu_visualizer',
            name='imu_visualizer_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
