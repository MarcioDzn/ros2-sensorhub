from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('sensorhub_bringup'),
        'config',
        'imu.yaml'
    ])
    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='imu_node',
            output='screen',
            parameters=[params_file]
        )
    ])