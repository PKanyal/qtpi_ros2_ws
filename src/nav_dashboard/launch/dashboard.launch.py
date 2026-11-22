"""ros2 launch nav_dashboard dashboard.launch.py [port:=8080] [allow_control:=true]"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("allow_control", default_value="true",
                              description="false = view-only page"),
        Node(package="nav_dashboard", executable="dashboard", name="nav_dashboard",
             output="screen",
             parameters=[{"port": LaunchConfiguration("port"),
                          "allow_control": LaunchConfiguration("allow_control")}]),
    ])
