from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    move_node = Node(
        package="kris_morg_py",
        executable="move_action_server_exe",
    )
    grip_node = Node(
        package="kris_morg_py",
        executable="gripper_action_server_exe"
    )
    ld.add_action(move_node)
    ld.add_action(grip_node)
    return ld
