from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_info',
            executable='camera_info_publisher',
            parameters=[{
                'camera_info_url': 'package://camera_info/config/test.yaml'
            }]
        ),

        Node(
            package='go2_media',
            executable='publish_front_image'
        ),

        Node(
            package='image_proc',
            executable="rectify_node",
            name='rectify_node',
            arguments=['image:=/camera/image_raw']
        )
    ])