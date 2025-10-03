from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Radar
        Node(
            package='darkhorse_perception',
            executable='radar_rx_node',
            name='radar_rx_node',
            output='screen',
            parameters=[{'can_interface': 'can1'}]
        ),

        # Speed from hall sensors
        Node(
            package='darkhorse_perception',
            executable='vehicle_speed_node',
            name='vehicle_speed_node',
            output='screen'
        ),

        # AEB (self-contained throttle + brake)
        Node(
            package='darkhorse_control',
            executable='aeb_node',
            name='aeb_node',
            output='screen'
        ),

        # HAL for Jetson
        Node(
            package='darkhorse_dbw',
            executable='hal_node_jetson',
            name='hal_node_jetson',
            output='screen'
        ),
    ])

