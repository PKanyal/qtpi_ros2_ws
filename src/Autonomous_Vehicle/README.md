# Autonomous Vehicle — ROS 2 Humble

ROS 2 Humble workspace for a differential-drive autonomous mobile robot, running on a Raspberry Pi 4B.

## Environment

- Hardware: Raspberry Pi 4 Model B + Arduino Uno (motor/encoder controller)
- Operating System: Ubuntu 22.04
- ROS 2 Distribution: Humble
- LiDAR: Slamtec SLLIDAR A2M12
- Drive system: Differential drive (L298N)
- Odometry: Wheel encoder-based
- Mapping: SLAM Toolbox

## Repository Structure

```text
Autonomous_Vehicle/
├── src/
│   ├── buggy_description/   # Robot URDF/Xacro
│   ├── encoder_odometry/    # Encoder-based odometry (/odom + odom->base_link TF)
│   ├── robot_bringup/       # Main bringup launch
│   └── sllidar_ros2/        # SLLIDAR driver
├── scripts/
│   └── motor_serial.py      # /cmd_vel <-> Arduino serial bridge
├── slam_config/
│   └── mapper_params_online_async.yaml
├── firmware/
│   └── velocity_controller/
│       └── velocity_controller.ino   # Arduino Uno PI velocity controller
└── README.md
```

## Installation

### 1. Clone the repository into a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/arushiy1300/Autonomous_Vehicle.git
cd ~/ros2_ws
```

### 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

### 3. Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
colcon build --symlink-install
```

### 5. Source the workspace

```bash
source install/setup.bash
```

## Robot Bringup

```bash
ros2 launch robot_bringup bringup.launch.py
```

Use `slam:=false` to start only the hardware without SLAM Toolbox.

The bringup launch file starts:

- Motor serial communication (Arduino on `/dev/ttyACM0`)
- Encoder-based odometry
- SLLIDAR A2M12 driver
- Static transform publisher (`base_link` → `laser`)
- SLAM Toolbox

## Navigation (Nav2)

Runs the robot hardware with `map_server` + AMCL + Nav2 on a saved map (SLAM is not started):

```bash
ros2 launch robot_bringup navigation.launch.py
ros2 launch robot_bringup navigation.launch.py map:=$HOME/ros2_ws/maps/buggy_map.yaml
```

Visualize and send goals:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

In RViz, set **2D Pose Estimate** first, then send a **Nav2 Goal**.

Nav2 parameters: `src/robot_bringup/config/nav2_params.yaml` (Regulated Pure Pursuit controller, max 0.18 m/s).

Odometry geometry can be tuned without editing code:

```bash
ros2 run encoder_odometry odom_node --ros-args -p wheel_base:=0.30 -p counts_per_rev:=4684.0
```

## Networking (CycloneDDS) and RViz on a Laptop

The Pi uses CycloneDDS (recommended for Nav2 on Humble) instead of the default FastDDS. `~/.bashrc` on the Pi sources:

```bash
source ~/ros2_ws/src/Autonomous_Vehicle/tools/ros_network.sh
```

It writes `~/.config/qtpi/cyclonedds.xml` with unicast peers, because phone hotspots and campus Wi-Fi often drop multicast. Peers come from `QTPI_PEERS`, `~/.config/qtpi/peers` (one IP per line), and the SSH client's IP.

Run RViz on a laptop instead of the Pi (RViz over remote desktop overloads the Pi). One-time setup on the laptop (ROS 2 Humble):

```bash
scp qtpi@<PI_IP>:ros2_ws/src/Autonomous_Vehicle/tools/setup_laptop_rviz.sh /tmp/
bash /tmp/setup_laptop_rviz.sh <PI_IP>
```

This installs CycloneDDS and the Nav2 RViz plugins if missing, registers the laptop's IP on the Pi, and adds an `rviz` command. Then, in a new terminal on the laptop:

```bash
rviz
```

RViz config: `src/robot_bringup/rviz/buggy.rviz`. If the laptop's IP changes, re-run the setup script.

## Crash / error reports

On the Pi (e.g. over SSH from the laptop), start the robot with the recorder instead of `ros2 launch`:

```bash
navrec                      # = tools/nav_record.sh -> navigation.launch.py
navrec map:=$HOME/ros2_ws/maps/buggy_map.yaml
navrec bringup.launch.py    # mapping
navrec --bag                # also record a rosbag of scan/odom/tf/amcl
```

However the run ends (Ctrl-C, a node crashing, the SSH session dropping) it writes `~/ros2_ws/reports/<date_time>/REPORT.md` (`~/ros2_ws/reports/latest` = newest run): detected problems, a timeline, warnings per node, topic rates, scan/TF delay, AMCL uncertainty, Pi temperature/throttling and Arduino/lidar USB drops. Run it inside `tmux` if the Wi-Fi is flaky, so an SSH drop does not stop the robot.

On the laptop, `rviz` (from `setup_laptop_rviz.sh`, re-run it to update) saves its own report and uploads it to the Pi on exit, where it is merged into the run's `REPORT.md`. If RViz freezes, run `rviz-hang` in another laptop terminal instead of force-quitting: it saves what every RViz thread was doing, then kills RViz.

Rebuild a report (e.g. after a late upload): `python3 tools/make_report.py [run_dir]`.

## Arduino Firmware

`firmware/velocity_controller/velocity_controller.ino` runs on the Arduino Uno: it drives the L298N with a per-wheel PI speed controller and reads both quadrature encoders. Flash it with the Arduino IDE or `arduino-cli` (board: Arduino Uno, 115200 baud).

## Arduino Serial Protocol

- Pi → Arduino: `V,<linear m/s>,<angular rad/s>` every 50 ms
- Arduino → Pi: `E,<left_count>,<right_count>` every 50 ms

## Coordinate Frames

```text
map
 └── odom
      └── base_link
           └── laser
```

## Mapping

The project uses SLAM Toolbox in online asynchronous mode.

Configuration file:

```text
slam_config/mapper_params_online_async.yaml
```

Save a generated occupancy grid:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/c205a
```

This generates:

- `c205a.yaml`
- `c205a.pgm`
