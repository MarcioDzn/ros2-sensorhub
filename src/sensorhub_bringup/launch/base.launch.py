from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='imu_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])