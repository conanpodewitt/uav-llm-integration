from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for handling LLM API integration, image processing, and publishing to /cmd_vel
    llm_node = Node(
        package='llm_integration',
        executable='llm_node',
        name='llm_node',
        output='screen'
    )

    # Node for receiving text input (CLI or GUI) and publishing on /ui
    ui_node = Node(
        package='llm_integration',
        executable='ui_node',
        name='ui_node',
        output='screen'
    )

    # Node for processing camera images and publishing captions on /camera_caption
    camera_caption_node = Node(
        package='llm_integration',
        executable='camera_caption_node',
        name='camera_caption_node',
        output='screen'
    )

    return LaunchDescription([
        llm_node,
        ui_node,
        camera_caption_node,
    ])
