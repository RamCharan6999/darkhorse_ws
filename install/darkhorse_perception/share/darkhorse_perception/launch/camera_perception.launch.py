#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    cam_id = '/dev/v4l/by-id/usb-Lenovo_Lenovo_Performance_Camera_145081A8-video-index0'

    video_dev_arg = DeclareLaunchArgument('video_device', default_value=cam_id)

    v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'pixel_format': 'YUYV',
            'image_size': [640, 480],
            'output_encoding': 'bgr8'
        }]
    )

    # model dir
    pkg_share = os.path.join(os.path.dirname(__file__), '..')
    model_dir = os.path.abspath(os.path.join(pkg_share, 'models'))

    cam = Node(
        package='darkhorse_perception',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'publish_debug': True,
            'frame_id': 'camera_optical',
            'tsd_model_path': os.path.join(model_dir, 'ts_yolov8n.onnx'),
            'tld_model_path': os.path.join(model_dir, 'tl_yolov8n.onnx'),
            'img_size': 640,
            'conf_thres': 0.30
        }]
    )

    return LaunchDescription([video_dev_arg, v4l2, cam])

