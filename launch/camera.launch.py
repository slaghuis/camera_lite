import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    camera_node=Node(
        package = 'camera_lite',
        name = 'camera',
        executable = 'camera_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"frame_width"  : 640},
            {"frame_height" : 480},
            {"device_id"    : 0},
            {"frequency"    : 15.0},
            {"frame_id"     : "camera"},
            {"reliability"  : "reliable"},
            {"history"      : "keep_last"},
            {"depth"        : 5}
        ]
    )
    picture_node=Node(
        package = 'camera_lite',
        name = 'picture_node',
        executable = 'picture_node',
        output="screen",
        emulate_tty=True
    )
    ld.add_action(camera_node)
    ld.add_action(picture_node)

    return ld
