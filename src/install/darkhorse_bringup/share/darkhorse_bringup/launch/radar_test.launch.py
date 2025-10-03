from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1. The Hardware Abstraction Layer
        Node(
            package='darkhorse_dbw',
            executable='hal_node',
            name='hal_node',
            output='screen'
        ),

        # 2. The Fake Speed Publisher to activate the radar
        Node(
            package='darkhorse_control',
            executable='fake_speed_publisher',
            name='fake_speed_publisher'
        ),

        # 3. The Radar Visualizer
        Node(
            package='darkhorse_perception',
            executable='radar_visualizer',
            name='radar_visualizer'
        ),

        # 4. The Static Transform Publisher to provide a 'base_link' frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),

        # 5. RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
