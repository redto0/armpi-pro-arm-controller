from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bus_servo_controller',
            executable='bus_servo_controller_node.py',
            name='bus_servo_controller',
            output='screen',
            parameters=[{
                'servo_ids': [1, 2, 3],  # Example with 3 servos
                'default_duration_ms': 1000,
                'publish_rate_hz': 10.0
            }]
        )
    ])
