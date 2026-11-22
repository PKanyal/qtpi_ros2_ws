"""View the buggy URDF on its own (no robot hardware needed).

    ros2 launch buggy_description display.launch.py
    ros2 launch buggy_description display.launch.py gui:=true   # wheel sliders
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("buggy_description"),
                "urdf",
                "buggy.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Show joint_state_publisher_gui sliders",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(gui),
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
        ),

        # No odom/map when viewing standalone, so use base_link.
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-f", "base_link"],
        ),
    ])
