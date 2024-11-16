import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for enabling/disabling recording
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Set to "true" to enable ROS bag recording'
        ),

        # Define the talker node with the frequency parameter
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='publisher_node',
            output='screen',
            # parameters=[{'publisher_frequency': LaunchConfiguration('freq')}],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    
        # Define the listener node
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='subscriber_node',
            output='screen'
        ),

        # ROS bag recorder with condition
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-o', 'rosbag_record', '-a'],
            output='screen'
        ),

        # Event handler to print a message when the publisher node starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=Node(
                    package='beginner_tutorials',
                    executable='talker',
                    name='publisher_node'
                ),
                on_start=[
                    ExecuteProcess(
                        cmd=['echo', 'Publisher node has started'],
                        output='screen'
                    )
                ]
            )
        )
    ])
