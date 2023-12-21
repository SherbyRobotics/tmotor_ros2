from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
        return LaunchDescription([
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('joy_input', default_value='js0'),
        
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{'deadzone': 0.0}],
            arguments=['dev', '/dev/input/', LaunchConfiguration('joy_input')],
        ),

        Node(
            package='tmotor_ros2',
            executable='pyro_2dof_controller',
            name='controller',
            output='screen',
        ),

        Node(
            package='tmotor_ros2',
            executable='tmotor_ros2',
            name='tmotors',
            parameters=[{'inverted': LaunchConfiguration('inverted')}],
        ),
        
    ])

