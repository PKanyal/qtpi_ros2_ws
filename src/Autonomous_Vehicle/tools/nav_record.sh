#!/usr/bin/env bash
# Run the robot (default: Nav2 on a saved map) on the Pi and ALWAYS leave an
# error report behind, whether it ends with Ctrl-C, a node crash, or the SSH
# session dropping.
#
# Usage (on the Pi, e.g. over SSH from the laptop):
#   ~/ros2_ws/src/Autonomous_Vehicle/tools/nav_record.sh                # navigation.launch.py
#   ~/ros2_ws/src/Autonomous_Vehicle/tools/nav_record.sh map:=$HOME/ros2_ws/maps/buggy_map.yaml
#   ~/ros2_ws/src/Autonomous_Vehicle/tools/nav_record.sh bringup.launch.py     # mapping
#   ~/ros2_ws/src/Autonomous_Vehicle/tools/nav_record.sh --bag ...      # also record a rosbag
#
# Each run goes to ~/ros2_ws/reports/<date_time>/ (~/ros2_ws/reports/latest
# points at the newest) and contains:
#   REPORT.md    summary + detected problems   <- read this first
#   console.log  everything the launch printed (incl. Python tracebacks)
#   ros_log/     every node's own log file
#   monitor.csv  1 Hz topic rates, scan/TF delay, AMCL pose + uncertainty
#   events.log   initial poses, goals, AMCL jumps, topics going silent
#   system.csv   CPU temp/frequency/load, Arduino + lidar USB presence
#   kernel.log   kernel messages during the run (USB disconnects etc.)
# RViz reports uploaded from the laptop land in ~/ros2_ws/reports/laptop/
# and are merged into REPORT.md. Rebuild a report any time with:
#   python3 ~/ros2_ws/src/Autonomous_Vehicle/tools/make_report.py [run_dir]

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${QTPI_WS:-$HOME/ros2_ws}"
REPORTS="${QTPI_REPORTS:-$WS/reports}"

BAG=0
if [ "${1:-}" = "--bag" ]; then BAG=1; shift; fi
LAUNCH_PKG="${QTPI_LAUNCH_PKG:-robot_bringup}"
LAUNCH_FILE="navigation.launch.py"
if [ $# -gt 0 ] && [[ "$1" == *.py ]]; then LAUNCH_FILE="$1"; shift; fi

# ROS setup scripts do not like `set -u`, so source them first.
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$TOOLS/ros_network.sh"
set -o pipefail

RUN="$REPORTS/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$RUN"
ln -sfn "$RUN" "$REPORTS/latest"
export ROS_LOG_DIR="$RUN/ros_log"
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

START_EPOCH=$(date +%s)
throttled() { cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>/dev/null || echo "?"; }

{
    echo "start_epoch=$START_EPOCH"
    echo "start=$(date '+%F %T %Z')"
    echo "host=$(hostname)  ip=$(hostname -I | awk '{print $1}')"
    echo "command=ros2 launch $LAUNCH_PKG $LAUNCH_FILE $*"
    echo "rmw=$RMW_IMPLEMENTATION  domain=${ROS_DOMAIN_ID:-0}"
    echo "dds_peers=$(grep -o 'Peer address="[^"]*"' "${CYCLONEDDS_URI#file://}" | cut -d'"' -f2 | tr '\n' ' ')"
    echo "ssh_client=${SSH_CLIENT%% *}"
    echo "git=$(git -C "$WS/src/Autonomous_Vehicle" rev-parse --short HEAD 2>/dev/null) dirty_files=$(git -C "$WS/src/Autonomous_Vehicle" status --porcelain 2>/dev/null | wc -l)"
    echo "throttled_start=$(throttled)"
    echo "uptime=$(uptime)"
} > "$RUN/meta.txt"

# --- background: system sampler (every 2 s) -------------------------------
sample_system() {
    local lidar="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_4d129d87a554b14eabedef55218d5403-if00-port0"
    echo "t,temp_c,cpu_mhz,load1,mem_used_pct,arduino_ttyACM0,lidar,top_cpu" > "$RUN/system.csv"
    while :; do
        local t temp freq load mem acm lid top
        t=$(date +%s.%N | cut -c1-14)
        temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
        freq=$(awk '{printf "%d", $1/1000}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)
        load=$(cut -d' ' -f1 /proc/loadavg)
        mem=$(free | awk '/^Mem:/ {printf "%.0f", ($2-$7)*100/$2}')
        [ -e /dev/ttyACM0 ] && acm=1 || acm=0
        [ -e "$lidar" ] && lid=1 || lid=0
        top=$(ps -eo pcpu,comm --sort=-pcpu --no-headers | head -4 \
              | awk '{printf "%s:%s ", $2, $1}')
        echo "$t,$temp,$freq,$load,$mem,$acm,$lid,$top" >> "$RUN/system.csv"
        sleep 2
    done
}
sample_system &
SAMPLER_PID=$!

# --- background: ROS topic / localization monitor --------------------------
python3 "$TOOLS/nav_monitor.py" "$RUN" > "$RUN/monitor_stdout.log" 2>&1 &
MONITOR_PID=$!

BAG_PID=""
if [ "$BAG" = 1 ]; then
    ros2 bag record -o "$RUN/bag" /scan /odom /tf /tf_static /amcl_pose \
        /initialpose /goal_pose /cmd_vel /encoder_counts \
        > "$RUN/bag_stdout.log" 2>&1 &
    BAG_PID=$!
fi

finish() {
    trap - EXIT INT TERM HUP
    echo
    echo "[nav_record] writing report ..."
    kill "$SAMPLER_PID" 2>/dev/null
    kill -INT "$MONITOR_PID" ${BAG_PID:+"$BAG_PID"} 2>/dev/null
    sleep 1
    kill "$MONITOR_PID" 2>/dev/null
    [ -n "$BAG_PID" ] && wait "$BAG_PID" 2>/dev/null
    {
        echo "end_epoch=$(date +%s)"
        echo "end=$(date '+%F %T %Z')"
        echo "launch_exit_code=${LAUNCH_RC:-?}"
        echo "ended_by=${ENDED_BY:-launch exited by itself}"
        echo "throttled_end=$(throttled)"
    } >> "$RUN/meta.txt"
    journalctl -k --no-pager -o short-iso --since "@$START_EPOCH" \
        > "$RUN/kernel.log" 2>&1
    python3 "$TOOLS/make_report.py" "$RUN" --print-summary
    # Huge logs (e.g. a serial error flood) eat the SD card: compress them.
    find "$RUN" -name '*.log' -size +5M -exec gzip -f {} \;
    echo "[nav_record] report: $RUN/REPORT.md"
}
trap finish EXIT
# Let the launch shut its nodes down itself; we only note why we stopped.
trap 'ENDED_BY="Ctrl-C"' INT
trap 'ENDED_BY="SIGTERM"' TERM
trap 'ENDED_BY="SSH session closed (SIGHUP)"' HUP

echo "[nav_record] run dir: $RUN"
ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" "$@" 2>&1 | tee -i --output-error=warn "$RUN/console.log"
LAUNCH_RC=${PIPESTATUS[0]}
