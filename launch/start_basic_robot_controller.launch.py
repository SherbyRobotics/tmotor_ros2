import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
    
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('joy_input', default_value='js0'),
        
        # joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{'deadzone': 0.0}],
            arguments=['dev:=/dev/input/js0']
        ),

        # basic_robot_controller.py
        Node(
            package='tmotor_ros2',
            executable='basic_2dof_controller',
            name='controller'
        ),

        # tmotor_ros.py
        Node(
            package='tmotor_ros2',
            executable='tmotor_ros2',
            name='tmotors',
            parameters=[{'inverted': LaunchConfiguration('inverted')}],
        ),
        
    ])

