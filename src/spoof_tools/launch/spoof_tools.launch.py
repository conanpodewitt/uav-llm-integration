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

    return LaunchDescription([
        poison_node,
    ])
