from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'folder_path',
            default_value='/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/2011_10_03/2011_10_03_drive_0027_sync', #<--- CHANGE ME
            description='Path to KITTI dataset'
        ),
        
        DeclareLaunchArgument(
            'oxts_data',
            default_value='/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/2011_10_03/2011_10_03_drive_0027_sync/oxts/data', #<--- CHANGE ME
            description='Path to GPS data'
        ),
        
        DeclareLaunchArgument(
            'true_pose',
            default_value='/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/data_odometry_poses/dataset/poses/00.txt', #<--- CHANGE ME
            description='Path to ground truth'
        ),
        
        Node(
            package='visual_odometry',
            executable='vo_main',
            name='vo_main',
            parameters=[{
                'folder_path': LaunchConfiguration('folder_path'),
                'oxts_data': LaunchConfiguration('oxts_data'),
                'true_pose': LaunchConfiguration('true_pose'),
                'frame_rate': 10.0,
                'min_num_features': 2000,
            }],
            output='screen'
        )
    ])