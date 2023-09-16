import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Launch your custom Python script
        Node(
            package='solution_package',
            executable='m_h_tc',
            name='m_h_tc_node'
        )
    ])
