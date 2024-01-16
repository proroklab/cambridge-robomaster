import socket
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = socket.gethostname().replace("-", "_") + '/camera_0'
    return LaunchDescription(
        [
            Node(
                package="camera",
                executable="camera_source",
                namespace=namespace,
                name="camera_source",
                parameters=[{
                    "camera_idx": 0,
                    "width": 1920,
                    "height": 1080,
                    "framerate": 15.0,
                    "camera_info_name": "front_camera",
                    "camera_info_url": "file:///opt/robomaster/src/camera/cfg/front_camera.yaml",
                }],
            ),
            Node(
                package="camera",
                executable="camera_proc",
                namespace=namespace,
                name="camera_proc",
                parameters=[{
                    "image_height": 224,
                    "fov": 120.0,
                    "aperture_width": 6.287,
                    "aperture_height": 4.712,
                }],
            ),
        ]
    )

