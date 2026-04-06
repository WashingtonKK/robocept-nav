"""Launch obstacle avoider only (for use with teleop)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robocept_nav'),
        'config', 'nav.yaml',
    )

    return LaunchDescription([
        Node(
            package='robocept_nav',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            namespace='robocept',
            parameters=[config],
            remappings=[
                ('/robocept/lidar/scan', '/robocept/lidar/scan'),
                ('/nav_cmd_vel', '/nav_cmd_vel'),
                ('/cmd_vel', '/cmd_vel'),
            ],
            output='screen',
        ),
    ])
