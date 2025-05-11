from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # PoisonNode intercepts /text_in → /text_in_orig → /text_in
    poison_node = Node(
        package='spoof_tools',
        executable='poison_node',
        name='poison_node',
        output='screen'
    )

    # SpoofNode intercepts camera and lidar
    spoof_node = Node(
        package='spoof_tools',
        executable='spoof_node',
        name='spoof_node',
        output='screen'
    )

    return LaunchDescription([
        poison_node,
        spoof_node,
    ])
