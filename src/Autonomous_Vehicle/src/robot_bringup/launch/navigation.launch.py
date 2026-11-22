"""Robot hardware + Nav2 on a saved map (map_server + AMCL).

Usage:
    ros2 launch robot_bringup navigation.launch.py                    # default map
    ros2 launch robot_bringup navigation.launch.py map:=h_second_floor
    ros2 launch robot_bringup navigation.launch.py map:=/full/path/to/map.yaml

A bare map name is looked up in ~/ros2_ws/maps/ (".yaml" is optional).

Then in RViz: set "2D Pose Estimate" first, then send a "2D Goal Pose".
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

MAPS_DIR = os.path.expanduser("~/ros2_ws/maps")
DEFAULT_MAP = "c205a"


def resolve_map(value):
    """Map name or path -> absolute path of the map .yaml file."""
    path = os.path.expanduser(value.strip())
    if os.sep not in path:
        path = os.path.join(MAPS_DIR, path)
    if not path.endswith(".yaml"):
        path += ".yaml"
    return os.path.abspath(path)


def available_maps():
    return sorted(os.path.splitext(os.path.basename(p))[0]
                  for p in glob.glob(os.path.join(MAPS_DIR, "*.yaml")))


def nav2_stack(context):
    map_path = resolve_map(LaunchConfiguration("map").perform(context))
    if not os.path.isfile(map_path):
        raise RuntimeError(
            f"Map not found: {map_path}\n"
            f"Maps in {MAPS_DIR}: {', '.join(available_maps()) or '(none)'}")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    return [
        LogInfo(msg=f"[navigation] using map {map_path}"),

        # map_server + AMCL + Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "map": map_path,
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "True",
            }.items(),
        ),
    ]


def generate_launch_description():

    bringup_dir = get_package_share_directory("robot_bringup")

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value=DEFAULT_MAP,
            description=(f"Map name in {MAPS_DIR} (e.g. h_second_floor) "
                         "or full path to a map .yaml"),
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "nav2_params.yaml"
            ),
            description="Full path to the Nav2 parameters file",
        ),

        DeclareLaunchArgument(
            "bridge",
            default_value="true",
            description="Start the Zenoh bridge for RViz in Docker",
        ),

        DeclareLaunchArgument(
            "lidar_yaw",
            default_value="3.0368729",
            description="Lidar yaw on the robot in radians (0.0 = facing forward)",
        ),

        # Motors, odometry, lidar, robot model, Zenoh bridge (no SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "bringup.launch.py")
            ),
            launch_arguments={
                "slam": "false",
                "bridge": LaunchConfiguration("bridge"),
                "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            }.items(),
        ),

        OpaqueFunction(function=nav2_stack),
    ])
