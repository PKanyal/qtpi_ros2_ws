import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bringup.paths import (
    DEFAULT_ARDUINO_PORT,
    DEFAULT_LIDAR_PORT,
    project_dir as find_project_dir,
)

# Zenoh bridge: lets RViz in Docker on the Mac connect over one TCP
# connection (port 7447) instead of DDS/UDP, which Docker's NAT blocks.
# Started here so it runs for both mapping and navigation.
# Must be the same version as in tools/docker_rviz/Dockerfile (1.10.2).
ZENOH_BRIDGE = os.path.expanduser("~/.local/bin/zenoh-bridge-ros2dds")
ZENOH_PORT = "7447"


def zenoh_bridge_actions():
    if not os.path.exists(ZENOH_BRIDGE):
        return [LogInfo(msg=(
            f"[bringup] {ZENOH_BRIDGE} not installed: Docker RViz "
            "cannot connect (see tools/docker_rviz/README.md)"))]

    return [ExecuteProcess(
        cmd=[
            ZENOH_BRIDGE,
            "-l", f"tcp/0.0.0.0:{ZENOH_PORT}",
            "--no-multicast-scouting",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("bridge")),
    )]


# Robot processes that must only run once. If an earlier launch died without
# cleaning up (e.g. its SSH session dropped), its children keep running:
# two odom_nodes publish conflicting odom->base_link TF (the robot "spins"
# in RViz) and two motor_serial.py fight over the Arduino serial port.
SINGLETON_PROCESSES = {
    "motor_serial.py": "scripts/motor_serial.py",
    "odom_node": "encoder_odometry/odom_node",
    "sllidar_node": "sllidar_ros2/sllidar_node",
    "robot_state_publisher": "robot_state_publisher/robot_state_publisher",
    "joint_state_publisher": "joint_state_publisher/joint_state_publisher",
    "zenoh bridge": "zenoh-bridge-ros2dds",
    "nav2 container": "component_container_isolated",
    "slam_toolbox": "slam_toolbox/async_slam_toolbox_node",
}


def find_leftover_robot_processes():
    """Return [(pid, name)] of robot processes already running."""
    me = os.getpid()
    found = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="ignore")
        except OSError:
            continue
        for name, pattern in SINGLETON_PROCESSES.items():
            if pattern in cmd:
                found.append((int(pid), name))
                break
    return sorted(found)


def check_no_leftovers():
    leftovers = find_leftover_robot_processes()
    if leftovers:
        pids = " ".join(str(p) for p, _ in leftovers)
        listing = "\n".join(f"  {p:>7}  {n}" for p, n in leftovers)
        raise RuntimeError(
            "Robot processes from an earlier launch are still running:\n"
            f"{listing}\n"
            "Running a second copy breaks odometry and the Arduino link.\n"
            f"Stop them with:   kill {pids}\n"
            "(or run this launch with check_leftovers:=false to skip the check)")


def generate_launch_description():

    argv = " ".join(os.sys.argv)
    skip = ("check_leftovers:=false" in argv
            or "--show-args" in argv or " -s" in argv
            or os.environ.get("BUGGY_SKIP_LEFTOVER_CHECK") == "1")
    if not skip:
        check_no_leftovers()

    # Found from where robot_bringup is installed (or $BUGGY_WS), so the
    # workspace does not have to live in ~/ros2_ws.
    project_dir = find_project_dir()

    slam_params = os.path.join(
        project_dir, "slam_config", "mapper_params_online_async.yaml"
    )

    motor_script = os.path.join(
        project_dir, "scripts", "motor_serial.py"
    )

    sllidar_launch = os.path.join(
        get_package_share_directory("sllidar_ros2"),
        "launch",
        "sllidar_a2m12_launch.py",
    )

    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("buggy_description"),
                "urdf",
                "buggy.urdf.xacro",
            ),
            " lidar_yaw:=",
            LaunchConfiguration("lidar_yaw"),
        ]),
        value_type=str,
    )

    return LaunchDescription([

        # slam:=false skips SLAM Toolbox (navigation.launch.py uses
        # map_server + AMCL instead).
        DeclareLaunchArgument(
            "slam",
            default_value="true",
            description="Run SLAM Toolbox for mapping",
        ),

        DeclareLaunchArgument(
            "check_leftovers",
            default_value="true",
            description="Refuse to start if robot processes from an earlier launch are running",
        ),

        DeclareLaunchArgument(
            "bridge",
            default_value="true",
            description="Start the Zenoh bridge for RViz in Docker",
        ),

        # Lidar mounting angle: +174 deg (final position, 2026-11-19).
        # C205A scan-to-map fit gave -6 / +174 deg (180 deg apart, nearly
        # equal fit). Physical checks decide it: a wall on the robot's right
        # and a person walking in FRONT of the robot both appear correctly
        # only with +174 (with -6 the person showed up behind).
        # Override with lidar_yaw:=<radians> (0.0 = lidar 0 deg forward).
        DeclareLaunchArgument(
            "lidar_yaw",
            default_value="3.0368729",
            description="Lidar yaw on the robot in radians (0.0 = facing forward)",
        ),

        DeclareLaunchArgument(
            "lidar_port",
            default_value=DEFAULT_LIDAR_PORT,
            description="Serial device of the SLLIDAR (see ls /dev/serial/by-id/)",
        ),

        DeclareLaunchArgument(
            "arduino_port",
            default_value=DEFAULT_ARDUINO_PORT,
            description="Serial device of the Arduino motor controller",
        ),

        *zenoh_bridge_actions(),

        # Arduino motor serial bridge
        ExecuteProcess(
            cmd=["python3", motor_script, "--ros-args",
                 "-p", ["port:=", LaunchConfiguration("arduino_port")]],
            output="screen",
        ),

        # Encoder odometry
        Node(
            package="encoder_odometry",
            executable="odom_node",
            output="screen",
            parameters=[{
                # Effective (skid-steer) wheel base from a 2-turn spin test
                # on 2026-11-18: 0.26 m gave 1031 deg for a real 720 deg.
                # Keep equal to WHEEL_SEPARATION in the Arduino firmware.
                "wheel_base": 0.372,
                "wheel_radius": 0.04,
                "counts_per_rev": 4684.0,
            }],
        ),

        # SLLIDAR A2M12
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": LaunchConfiguration("lidar_port"),
                "serial_baudrate": "256000",
                "frame_id": "laser",
                "scan_mode": "Sensitivity",
            }.items(),
        ),

        # Robot model: publishes /robot_description and the
        # base_link -> laser / wheel transforms from the URDF.
        # (Replaces the old static base_link -> laser publisher;
        # the lidar position now lives in mobile_base_buggy.xacro.)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Wheel joint states (zero angles) so the wheels have TF.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam_params_file": slam_params,
            }.items(),
            condition=IfCondition(LaunchConfiguration("slam")),
        ),
    ])
