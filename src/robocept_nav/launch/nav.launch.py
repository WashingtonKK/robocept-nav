"""Launch full navigation stack: waypoint nav + obstacle avoidance."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robocept_nav'),
        'config', 'nav.yaml',
    )

    obstacle_avoider = Node(
        package='robocept_nav',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        namespace='robocept',
        parameters=[config],
        output='screen',
    )

    waypoint_nav = Node(
        package='robocept_nav',
        executable='waypoint_nav',
        name='waypoint_nav',
        namespace='robocept',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        obstacle_avoider,
        waypoint_nav,
    ])
