#!/usr/bin/env bash
# Start the Nav2 web dashboard with the same ROS/DDS setup as the robot.
# Usage: src/nav_dashboard/run_dashboard.sh [port:=8080] [allow_control:=false]
# Works wherever the workspace is cloned: <ws>/src/nav_dashboard/run_dashboard.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${BUGGY_WS:-$(cd "$HERE/../.." && pwd)}"

source /opt/ros/humble/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "run_dashboard: $WS/install/setup.bash not found - build first:" >&2
    echo "  cd $WS && colcon build" >&2
    exit 1
fi
source "$WS/install/setup.bash"
# same CycloneDDS config the rest of the robot uses
[ -f "$WS/src/Autonomous_Vehicle/tools/ros_network.sh" ] && \
    source "$WS/src/Autonomous_Vehicle/tools/ros_network.sh"
exec ros2 launch nav_dashboard dashboard.launch.py "$@"
