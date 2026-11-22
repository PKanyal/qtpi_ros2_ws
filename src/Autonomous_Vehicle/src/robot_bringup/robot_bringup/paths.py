"""Locate the colcon workspace this package was built in.

Lets the launch files find maps/, slam_config/ and scripts/ wherever the
workspace was cloned, instead of assuming ~/ros2_ws.

Order:
  1. $BUGGY_WS, if set
  2. the parent of the "install" directory robot_bringup is installed in
     (works for normal, --symlink-install and --merge-install builds)
  3. ~/ros2_ws
"""

import os

from ament_index_python.packages import get_package_share_directory


def workspace_dir():
    env = os.environ.get("BUGGY_WS")
    if env:
        return os.path.abspath(os.path.expanduser(env))

    path = get_package_share_directory("robot_bringup")
    while True:
        parent = os.path.dirname(path)
        if parent == path:
            break
        if os.path.basename(path) == "install":
            return parent
        path = parent

    return os.path.expanduser("~/ros2_ws")


def project_dir():
    """src/Autonomous_Vehicle inside the workspace."""
    return os.path.join(workspace_dir(), "src", "Autonomous_Vehicle")


def maps_dir():
    """Saved maps; override with $BUGGY_MAPS."""
    env = os.environ.get("BUGGY_MAPS")
    if env:
        return os.path.abspath(os.path.expanduser(env))
    return os.path.join(workspace_dir(), "maps")


# Hardware defaults for this buggy; override with lidar_port:= / arduino_port:=
# on bringup.launch.py or navigation.launch.py. List devices with
#   ls /dev/serial/by-id/
DEFAULT_LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
    "4d129d87a554b14eabedef55218d5403-if00-port0"
)
DEFAULT_ARDUINO_PORT = "/dev/ttyACM0"
