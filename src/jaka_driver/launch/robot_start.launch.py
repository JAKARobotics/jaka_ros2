import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'ip' argument
        DeclareLaunchArgument('ip', default_value='10.5.5.100', description='IP address'),

        # Print the IP to the log for debugging
        LogInfo(
            msg=["The IP address is: ", LaunchConfiguration('ip')]  # Correct substitution usage
        ),

        # Launch the 'jaka_driver' node with the provided 'ip' parameter
        Node(
            package='jaka_driver',
            executable='jaka_driver',  # the executable to run
            name='jaka_driver',
            output='screen',
            parameters=[{'ip': LaunchConfiguration('ip')}],  # pass the 'ip' parameter
        ),
    ])
