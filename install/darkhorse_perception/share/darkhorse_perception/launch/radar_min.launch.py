from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.getenv('HOME'),
        'DH_ws/install/darkhorse_dbw/share/darkhorse_dbw/config'
    )

    return LaunchDescription([
        # Radar node
        Node(
            package='darkhorse_perception',
            executable='radar_rx_node',
            name='radar_rx_node',
            output='screen',
            parameters=[{
                'can_interface': 'can0',
                'poll_hz': 25.0,
                'frame_id': 'radar_link',
                'dbc_path': os.path.join(pkg_share, 'Starkenn_Orion_Radar_x_aBAJA.dbc')
            }]
        ),

        # RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.getenv('HOME'),
                'DH_ws/src/darkhorse_perception/config/radar.rviz'
            )]
        )
    ])

