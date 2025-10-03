from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    can_if   = LaunchConfiguration('can_interface')
    use_dbc  = LaunchConfiguration('use_dbc')
    poll_hz  = LaunchConfiguration('poll_hz')
    max_rng  = LaunchConfiguration('max_range_m')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('use_dbc', default_value='true'),
        DeclareLaunchArgument('poll_hz', default_value='25.0'),
        DeclareLaunchArgument('max_range_m', default_value='120.0'),
        DeclareLaunchArgument('frame_id', default_value='radar_link'),

        Node(
            package='darkhorse_perception',
            executable='radar_rx_node',
            name='radar_rx_node',
            output='screen',
            parameters=[{
                'can_interface': can_if,
                'use_dbc': use_dbc,
                'poll_hz': poll_hz,
                'max_range_m': max_rng,
                'frame_id': frame_id,
                'publish_points2': True,
                'publish_markers': True,
                'speed_topic': '/VehicleSpeed',   # optional
            }]
        ),

        # Static TF if you want to visualize in base_link:
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_tf',
            arguments=['0','0','0','0','0','0','base_link','radar_link'],
            output='screen'
        ),
    ])

