from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # PoisonNode intercepts /text_in
    poison_node = Node(
        package='spoof_tools',
        executable='poison_node',
        name='poison_node',
        output='screen'
    )

    # MirageNode intercepts /camera
    mirage_node = Node(
        package='spoof_tools',
        executable='mirage_node',
        name='mirage_node',
        output='screen'
    )

    return LaunchDescription([
        poison_node,
        mirage_node,
    ])
