from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('visual_odometry')
    
    # Declare arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2', #<--- CHANGE ME
        description='Camera device path'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file://<path to your camera calibration file eg- camera_info.yaml>', #<--- CHANGE ME
        description='Path to camera calibration file'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='10.0',
        description='Camera frame rate'
    )
    
    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            # 'image_width': 640,
            # 'image_height': 480,
            # 'pixel_format': 'yuyv',
            'camera_frame_id': 'narrow_stereo',
            # 'io_method': 'mmap',
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'usb_cam:=warn']
    )
    
    # Visual Odometry Node
    vo_node = Node(
        package='visual_odometry',
        executable='vo_main',
        name='vo_main',
        parameters=[{
            'use_camera': True,  # Subscribe to camera topic
            'min_num_features': 2000,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        video_device_arg,
        camera_info_url_arg,
        frame_rate_arg,
        usb_cam_node,
        vo_node
    ])