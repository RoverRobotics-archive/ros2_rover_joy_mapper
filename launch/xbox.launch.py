"""
Interfaces via USB with the hardware of the robot.
Drives the rover in response to teleop messages and publishes odometry from motor encoders.
Runs LIDAR sensor to scan visible obstacles.
Runs IMU to measure orientation and acceleration.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = Path(get_package_share_directory('openrover_joy_mapper'), 'config', 'controller.yaml')
    assert controller_config.is_file()
    topics_config = Path(get_package_share_directory('openrover_joy_mapper'), 'config', 'topics.yaml')
    assert topics_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='openrover_joy_mapper', node_executable='mapper_node.py', output='screen',
            # parameters=[controller_config, topics_config]
        ),
        Node(
            package='joy', node_executable='joy_node', output='screen',
            # parameters=[controller_config, topics_config]
        ),
    ])
