from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='one_hand_plus',
            executable='plastic',
            name='plastic_node',
            output='screen'
        ),
        Node(
            package='one_hand_plus',
            executable='glass',
            name='glass_node',
            output='screen'
        ),
        Node(
            package='one_hand_plus',
            executable='bread',
            name='bread_node',
            output='screen'
        ),
        Node(
            package='one_hand_plus',
            executable='vegetable',
            name='vegetable_node',
            output='screen'
        ),
        Node(
            package='one_hand_plus',
            executable='ui',
            name='ui_node',
            output='screen'
        )
    ])
