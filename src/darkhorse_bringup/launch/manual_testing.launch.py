from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Launches the Hardware Abstraction Layer
        Node(
            package='darkhorse_dbw',
            executable='hal_node',
            name='hal_node',
            output='screen'
        ),

        # Launches the Joystick driver using the hidraw backend
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {'device_path': '/dev/input/event13'},
                {'joy_type': 'hidraw'}
            ]
        ),

        # Launches the teleop node with parameters set directly
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'require_enable_button': False,
                'axis_linear': {'x': 1},   # Left stick up/down
                'axis_angular': {'yaw': 2} # Right stick left/right
            }],
            remappings=[('/cmd_vel', '/vehicle_control')]
        ),
    ])
