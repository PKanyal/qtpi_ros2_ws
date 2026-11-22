#!/usr/bin/env bash
# Start the Nav2 web dashboard with the same ROS/DDS setup as the robot.
# Usage: ~/nav_dashboard_ws/run_dashboard.sh [port:=8080] [allow_control:=false]
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
# same CycloneDDS config the rest of the robot uses (read-only use of the repo)
[ -f ~/ros2_ws/src/Autonomous_Vehicle/tools/ros_network.sh ] && \
    source ~/ros2_ws/src/Autonomous_Vehicle/tools/ros_network.sh
source ~/nav_dashboard_ws/install/setup.bash
exec ros2 launch nav_dashboard dashboard.launch.py "$@"
