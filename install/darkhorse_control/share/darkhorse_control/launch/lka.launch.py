#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Force Lenovo camera into YUYV @ 640x480
        ExecuteProcess(
            cmd=['v4l2-ctl','-d','/dev/video2',
                 '--set-fmt-video=width=640,height=480,pixelformat=YUYV'],
            output='screen'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'video_device': '/dev/video2',   # <-- pick Lenovo directly
                'pixel_format': 'YUYV',
                'image_size': [640,480],
                'output_encoding': 'yuv422_yuy2'
            }],
            remappings=[('/image_raw','/RGBImage')],
            output='screen'
        ),

        Node(
            package='darkhorse_control',
            executable='lka_node',
            name='lka_node',
            output='screen'
        )
    ])

