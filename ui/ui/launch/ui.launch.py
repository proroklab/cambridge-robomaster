import socket
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = socket.gethostname().replace("-", "_")
    return LaunchDescription(
        [
            Node(
                package="ui",
                executable="ui",
                namespace=namespace,
                name="user_interface",
            ),
        ]
    )
