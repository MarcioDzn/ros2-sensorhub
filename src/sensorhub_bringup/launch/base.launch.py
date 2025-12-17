from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    imu_params = PathJoinSubstitution([
        FindPackageShare('sensorhub_bringup'),
        'config',
        'imu.yaml'
    ])

    actuator_params = PathJoinSubstitution([
        FindPackageShare('sensorhub_bringup'),
        'config',
        'actuator.yaml'
    ])

    pressure_params = PathJoinSubstitution([
        FindPackageShare('sensorhub_bringup'),
        'config',
        'pressure.yaml'
    ])

    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='imu_node',
            output='screen',
            parameters=[imu_params]
        ),

        Node(
            package='pressure_driver',
            executable='pressure_node',
            output='screen',
            parameters=[pressure_params]
        ),
        
        Node(
            package='actuator_driver',
            executable='actuator_node',
            output='screen',
            parameters=[actuator_params]
        ),

        Node(
            package='sensor_sync',
            executable='sync_node',
            output='screen',
        )
    ])
