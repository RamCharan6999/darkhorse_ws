from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera driver node
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="usb_camera",
            output="screen",
            parameters=[{
                "video_device": "/dev/video0",   # change to /dev/video1 if needed
                "image_size": [640, 480],
                "pixel_format": "YUYV"
            }]
        ),

        # Perception node
        Node(
            package="darkhorse_perception",
            executable="camera_perception_node",
            name="camera_perception",
            output="screen",
            parameters=[{
                "model_path": "/home/darkhorse/DH_ws/src/darkhorse_perception/my_traffic_model.pt"
            }]
        )
    ])

