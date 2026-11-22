#!/usr/bin/env bash
# One-time setup on the LAPTOP that runs RViz for the buggy.
#
# After this, typing `rviz` in a new terminal on the laptop opens RViz with
# the Nav2 view, talking to the Pi over the network. Everything else (robot,
# map, Nav2) still runs on the Pi.
#
# Usage (on the laptop):
#   scp qtpi@<PI_IP>:ros2_ws/src/Autonomous_Vehicle/tools/setup_laptop_rviz.sh /tmp/
#   bash /tmp/setup_laptop_rviz.sh <PI_IP>
#
# Re-run it any time to pull updated config from the Pi.

set -euo pipefail

PI_ADDR="${1:-qtpi.local}"
PI_USER="${PI_USER:-qtpi}"
REPO="ros2_ws/src/Autonomous_Vehicle"
CFG="${XDG_CONFIG_HOME:-$HOME/.config}/qtpi"
BASHRC="$HOME/.bashrc"

echo "== Buggy RViz setup (Pi: $PI_USER@$PI_ADDR)"

# 1. ROS 2 Humble present?
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: /opt/ros/humble not found. Install ROS 2 Humble first." >&2
    exit 1
fi

# 2. Packages RViz needs for this setup.
missing=()
[ -e /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ] \
    || missing+=(ros-humble-rmw-cyclonedds-cpp)
[ -d /opt/ros/humble/share/nav2_rviz_plugins ] \
    || missing+=(ros-humble-nav2-rviz-plugins)
[ -x /opt/ros/humble/lib/rviz2/rviz2 ] \
    || missing+=(ros-humble-rviz2)

if [ ${#missing[@]} -gt 0 ]; then
    echo "Missing packages: ${missing[*]}"
    read -r -p "Install them now with sudo apt? [Y/n] " ans
    if [[ "${ans:-Y}" =~ ^[Yy]$ ]]; then
        sudo apt update
        sudo apt install -y "${missing[@]}"
    else
        echo "ERROR: cannot continue without: ${missing[*]}" >&2
        exit 1
    fi
fi

# 3. One SSH call (one password prompt):
#    - register this laptop's IP as a DDS peer on the Pi
#    - fetch the RViz config and network script
mkdir -p "$CFG"
pi_ip=$(getent ahostsv4 "$PI_ADDR" 2>/dev/null | awk '{print $1; exit}' || true)
laptop_ip=$(ip -4 route get "${pi_ip:-$PI_ADDR}" 2>/dev/null \
            | awk '{for (i = 1; i < NF; i++) if ($i == "src") {print $(i+1); exit}}')
if [ -z "$laptop_ip" ]; then
    echo "ERROR: cannot reach $PI_ADDR - are both on the same network?" >&2
    exit 1
fi
echo "Laptop IP: $laptop_ip   Pi IP: ${pi_ip:-?}"

# SSH key: lets `rviz` upload its crash report to the Pi without a password.
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_USER@$PI_ADDR" true 2>/dev/null; then
    read -r -p "Set up an SSH key for $PI_USER@$PI_ADDR (no more password prompts)? [Y/n] " ans
    if [[ "${ans:-Y}" =~ ^[Yy]$ ]]; then
        [ -f "$HOME/.ssh/id_ed25519.pub" ] || ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
        ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" "$PI_USER@$PI_ADDR"
    fi
fi
echo "Registering laptop on the Pi and fetching config..."
ssh "$PI_USER@$PI_ADDR" "
    mkdir -p ~/.config/qtpi &&
    touch ~/.config/qtpi/peers &&
    { grep -qx '$laptop_ip' ~/.config/qtpi/peers || echo '$laptop_ip' >> ~/.config/qtpi/peers; } &&
    tar -C $REPO -cf - tools/ros_network.sh src/robot_bringup/rviz/buggy.rviz
" | tar -xf - -C "$CFG"
mv -f "$CFG/tools/ros_network.sh" "$CFG/ros_network.sh"
mv -f "$CFG/src/robot_bringup/rviz/buggy.rviz" "$CFG/buggy.rviz"
rm -rf "$CFG/tools" "$CFG/src"

# 4. Remember how to reach the Pi: hostname plus its current IP as fallback.
if [[ "$PI_ADDR" =~ ^[0-9.]+$ ]]; then
    echo "qtpi.local $PI_ADDR" > "$CFG/pi_address"
else
    echo "$PI_ADDR ${pi_ip:-}" > "$CFG/pi_address"
fi
echo "$PI_USER" > "$CFG/pi_user"
echo "Pi address saved: $(cat "$CFG/pi_address")"

# 5. Install the `rviz` / `rviz-hang` commands into ~/.bashrc (replaces an
#    older copy).
start='# >>> buggy rviz >>>'
end='# <<< buggy rviz <<<'
if grep -qF "$start" "$BASHRC" 2>/dev/null; then
    sed -i "/$start/,/$end/d" "$BASHRC"
fi
cat >> "$BASHRC" <<'EOF'
# >>> buggy rviz >>>
# `rviz` opens RViz for the buggy (Nav2 view) over the network. Every run
# saves a report in ~/.config/qtpi/reports/ and uploads it to the Pi
# (~/ros2_ws/reports/laptop/) when RViz exits or crashes.
# If RViz FREEZES: run `rviz-hang` in another terminal instead of
# force-quitting - it saves what every RViz thread was doing, then kills it.
# Pi address: ~/.config/qtpi/pi_address   Setup: tools/setup_laptop_rviz.sh
unalias rviz 2>/dev/null
rviz() (
    cfg="${XDG_CONFIG_HOME:-$HOME/.config}/qtpi"
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    QTPI_PEERS="$(cat "$cfg/pi_address" 2>/dev/null)"
    source "$cfg/ros_network.sh" || exit 1
    pi=$(awk '{print ($2 != "" ? $2 : $1)}' "$cfg/pi_address")
    pi_user=$(cat "$cfg/pi_user" 2>/dev/null || echo qtpi)
    run="$cfg/reports/rviz_$(date +%Y-%m-%d_%H-%M-%S)"
    mkdir -p "$run"
    export ROS_LOG_DIR="$run/ros_log" RCUTILS_COLORIZED_OUTPUT=0
    start=$(date +%s)
    {
        echo "start_epoch=$start"
        echo "start=$(date '+%F %T %Z')"
        echo "laptop_ip=$(ip -4 route get "$pi" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") {print $(i+1); exit}}')"
        echo "pi=$pi_user@$pi"
        echo "rmw=$RMW_IMPLEMENTATION peers=$QTPI_PEERS"
        echo "rviz_display_server=${XDG_SESSION_TYPE:-?} DISPLAY=${DISPLAY:-} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
        echo "gpu=$(glxinfo -B 2>/dev/null | sed -n 's/.*OpenGL renderer string: //p' | head -1)$(command -v glxinfo >/dev/null || lspci 2>/dev/null | grep -m1 -iE 'vga|3d' | cut -d: -f3)"
        echo "mem_total_mb=$(free -m | awk '/^Mem:/ {print $2}')"
    } > "$run/meta.txt"
    echo "[rviz] $RMW_IMPLEMENTATION, Pi peers: $QTPI_PEERS"
    echo "[rviz] report folder: $run"

    rviz2 -d "$cfg/buggy.rviz" "$@" > "$run/console.log" 2>&1 &
    pid=$!
    echo "$pid $run" > "$cfg/rviz.running"
    trap 'kill -INT $pid 2>/dev/null' INT TERM
    tail -n +1 -f --pid=$pid "$run/console.log" &

    # every 2 s: RViz state, CPU (instant), memory, threads, ping to the Pi
    (
        hz=$(getconf CLK_TCK); prev=""; idle=0; dumps=0
        echo "t,state,cpu_pct,rss_mb,threads,ping_ms"
        while [ -r /proc/$pid/stat ]; do
            read -r -a f < /proc/$pid/stat
            ticks=$(( f[13] + f[14] ))
            cpu=""; [ -n "$prev" ] && cpu=$(( (ticks - prev) * 100 / (hz * 2) ))
            prev=$ticks
            rss=$(awk '/^VmRSS/ {print int($2/1024)}' /proc/$pid/status 2>/dev/null)
            thr=$(awk '/^Threads/ {print $2}' /proc/$pid/status 2>/dev/null)
            p=$(ping -c1 -W1 "$pi" 2>/dev/null | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
            echo "$(date +%s),${f[2]},$cpu,$rss,$thr,${p:-lost}"
            # A displaying RViz always uses some CPU; ~0% for 6 s means it
            # is blocked (frozen). Save what each thread waits on (no root
            # needed), up to 3 times, so a force-quit still leaves evidence.
            if [ -n "$cpu" ] && [ "$cpu" -le 1 ]; then idle=$((idle + 1)); else idle=0; fi
            if [ "$idle" -ge 3 ] && [ "$dumps" -lt 3 ]; then
                dumps=$((dumps + 1)); idle=0
                {
                    date
                    echo "--- tid name state kernel-wait (auto: RViz at ~0% CPU for 6 s)"
                    for t in /proc/$pid/task/*; do
                        printf '%s %s %s %s\n' "${t##*/}" "$(cat "$t/comm")" \
                            "$(awk '{print $3}' "$t/stat")" "$(cat "$t/wchan" 2>/dev/null)"
                    done
                } > "$run/auto_threads_$dumps.txt" 2>&1
            fi
            sleep 2
        done
    ) > "$run/samples.csv" 2>/dev/null &

    rc=0
    wait $pid; rc=$?
    while kill -0 $pid 2>/dev/null; do wait $pid; rc=$?; done
    sleep 0.3

    case $rc in
        0)   meaning="closed normally" ;;
        130) meaning="stopped with Ctrl-C" ;;
        137) meaning="SIGKILL: force-quit, or killed by the out-of-memory killer" ;;
        139) meaning="CRASH: segmentation fault" ;;
        134) meaning="CRASH: abort (uncaught exception / assertion)" ;;
        135) meaning="CRASH: bus error" ;;
        143) meaning="SIGTERM (closed by the system / logout)" ;;
        *)   meaning="exited with error" ;;
    esac
    [ -f "$run/hang_threads.txt" ] && meaning="FROZE: killed by rviz-hang (thread dump saved)"
    {
        echo "end_epoch=$(date +%s)"
        echo "end=$(date '+%F %T %Z')"
        echo "exit_code=$rc"
        echo "exit_meaning=$meaning"
    } >> "$run/meta.txt"
    journalctl -k --no-pager --since "@$start" 2>/dev/null \
        | grep -iE 'out of memory|oom-kill|killed process|rviz|segfault|gpu hang|ring .*timeout|gpu reset|(amdgpu|i915|nouveau|nvidia).*(error|fault|timeout|hang|reset)' \
        > "$run/kernel.log"
    ls -l /var/crash/*rviz2* 2>/dev/null > "$run/system_crash_files.txt"
    rm -f "$cfg/rviz.running"
    echo "[rviz] exit code $rc: $meaning"

    echo "[rviz] uploading report to the Pi ..."
    if tar -C "$cfg/reports" -cf - "$(basename "$run")" \
        | ssh -o ConnectTimeout=5 "$pi_user@$pi" '
            mkdir -p ~/ros2_ws/reports/laptop &&
            tar -xf - -C ~/ros2_ws/reports/laptop &&
            { [ -e ~/ros2_ws/reports/latest ] &&
              python3 ~/ros2_ws/src/Autonomous_Vehicle/tools/make_report.py >/dev/null 2>&1; true; }'
    then
        echo "[rviz] uploaded: ~/ros2_ws/reports/laptop/$(basename "$run") on the Pi"
    else
        echo "[rviz] upload FAILED - report is still here: $run"
    fi
    exit $rc
)

# Run this (in another terminal) when RViz is frozen, instead of force-quit.
rviz-hang() (
    cfg="${XDG_CONFIG_HOME:-$HOME/.config}/qtpi"
    read -r pid run < "$cfg/rviz.running" 2>/dev/null \
        || { echo "no rviz started with the 'rviz' command is running"; exit 1; }
    kill -0 "$pid" 2>/dev/null || { echo "rviz (pid $pid) is not running"; exit 1; }
    out="$run/hang_threads.txt"
    {
        date
        echo "--- per thread: tid name state(R=running S=sleeping D=stuck in I/O) kernel-wait"
        for t in /proc/$pid/task/*; do
            printf '%s %s %s %s\n' "${t##*/}" "$(cat "$t/comm")" \
                "$(awk '{print $3}' "$t/stat")" "$(cat "$t/wchan" 2>/dev/null)"
        done
        if command -v gdb >/dev/null; then
            echo "--- gdb backtrace of all threads"
            sudo gdb -p "$pid" -batch -ex 'thread apply all bt 15' 2>&1 \
                | grep -E '^Thread|^#' | head -500
        else
            echo "--- (install gdb for full backtraces: sudo apt install gdb)"
        fi
    } > "$out" 2>&1
    echo "saved $out - killing rviz (pid $pid)"
    kill -9 "$pid"
)
# <<< buggy rviz <<<
EOF

echo
echo "Done. Open a NEW terminal and type:  rviz"
echo "If RViz freezes, run  rviz-hang  in another terminal (saves a thread dump)."
echo "If the Pi's IP changes, re-run:  bash $0 <NEW_PI_IP>"
