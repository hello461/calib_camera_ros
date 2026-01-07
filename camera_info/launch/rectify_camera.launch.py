from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publish camera information
        Node(
            package='camera_info',
            executable='camera_info_publisher',
            parameters=[{
                'camera_info_url': 'package://camera_info/config/camera_config.yaml'
            }]
        ),

        # Publish front image
        Node(
            package='go2_media',
            executable='publish_front_image'
        ),

        # Publish image rectified
        Node(
            package='image_proc',
            executable="rectify_node",
            name='rectify_node',
            arguments=['image:=/camera/image_raw']
        ),

        # View image raw from front camera
        Node(
            package="image_view",
            executable="image_view",
            name='Image_raw',
            remappings=[
                ('image', 'camera/image_raw')
            ]
        ),
        
        # View image rectify from camera
        Node(
            package='image_view',
            executable='image_view',
            name='Image_rectify',
            remappings=[
                ('image', 'image_rect')
            ]
        )
    ])