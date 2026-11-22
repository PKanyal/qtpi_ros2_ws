# qtpi buggy — ROS 2 Humble workspace

Complete colcon workspace for the qtpi autonomous buggy (Raspberry Pi 4B +
Arduino Uno + SLLIDAR A2M12): hardware bringup, SLAM, Nav2 navigation on saved
maps, and a web dashboard.

```text
<workspace>/
├── src/
│   ├── Autonomous_Vehicle/     robot packages, firmware, tools (see its README)
│   │   └── src/robot_bringup/  bringup.launch.py, navigation.launch.py, nav2_params.yaml
│   └── nav_dashboard/          Nav2 web dashboard (see its README)
├── maps/                       saved maps (navigation.launch.py map:=<name>)
└── reports/                    run reports written by tools/nav_record.sh
```

The workspace can be cloned anywhere: launch files find `maps/`, `slam_config/`
and `scripts/` from where the packages are installed. Set `BUGGY_WS=/path` to
force a workspace, or `BUGGY_MAPS=/path` for a different maps folder.

## Setup (Ubuntu 22.04)

```bash
# 1. ROS 2 Humble (if not installed): https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-ros-base python3-colcon-common-extensions python3-rosdep

# 2. Clone
git clone git@github.com:Pkanyal/qtpi_ros2_ws.git ~/ros2_ws
cd ~/ros2_ws

# 3. Dependencies (Nav2, SLAM Toolbox, xacro, pyserial, numpy, ...)
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install
source install/setup.bash        # add to ~/.bashrc to make it permanent

# 5. Serial port access (log out and back in afterwards)
sudo usermod -aG dialout $USER
```

Flash `src/Autonomous_Vehicle/firmware/velocity_controller/` to the Arduino Uno
(see `firmware/MDD10A_WIRING.md` / `velocity_controller_l298n/` for the motor
driver you have).

## Hardware ports

Defaults match the original buggy. On another robot, find your devices with

```bash
ls /dev/serial/by-id/        # lidar (CP2102) and Arduino
```

and pass them to either launch file:

```bash
ros2 launch robot_bringup navigation.launch.py \
    lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._if00-port0 \
    arduino_port:=/dev/ttyACM0
```

Robot-specific values that may also need changing on a different robot:
`lidar_yaw` (launch argument), wheel_base / wheel_radius / counts_per_rev in
`bringup.launch.py`, and the URDF in `buggy_description`.

## Run

```bash
# Mapping (SLAM Toolbox)
ros2 launch robot_bringup bringup.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/<name>   # save the map

# Navigation on a saved map (map_server + AMCL + Nav2)
ros2 launch robot_bringup navigation.launch.py map:=c205a

# Same, but always leave a run report in reports/
src/Autonomous_Vehicle/tools/nav_record.sh map:=c205a

# Web dashboard (in another terminal, with Nav2 or SLAM running)
src/nav_dashboard/run_dashboard.sh            # http://<robot-ip>:8080/
```

Useful launch arguments: `bridge:=false` (no Zenoh bridge),
`check_leftovers:=false`, `lidar_yaw:=<rad>`, `params_file:=<nav2 yaml>`.

## Optional extras (not in this repo)

- **Zenoh bridge** for RViz in Docker on a Mac: install
  `zenoh-bridge-ros2dds` 1.10.2 to `~/.local/bin/` (see
  `src/Autonomous_Vehicle/tools/docker_rviz/README.md`). Without it bringup
  just logs a notice.
- **Laptop RViz over DDS**: run `src/Autonomous_Vehicle/tools/setup_laptop_rviz.sh`
  on the laptop; peers are stored in `~/.config/qtpi/peers` on the robot.
