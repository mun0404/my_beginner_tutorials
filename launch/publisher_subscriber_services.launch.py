import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Define the talker node with the frequency parameter
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='publisher_node',
            output='screen',
            parameters=[{'publisher_frequency': launch.substitutions.LaunchConfiguration('freq', default=2.0)}],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    
        # Define the listener node
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='subscriber_node',
            output='screen'
        ),
    ])
