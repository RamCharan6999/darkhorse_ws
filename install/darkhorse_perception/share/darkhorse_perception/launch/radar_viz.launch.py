from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('darkhorse_perception'),
        'config',
        'radar.yaml'
    ])

    return LaunchDescription([
        # Radar node
        Node(
            package='darkhorse_perception',
            executable='radar_rx_node',
            name='radar_rx_node',
            parameters=[params],
            output='screen'
        ),

        # Static TF: world -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['0','0','0','0','0','0','world','base_link']
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])

