# ROS 2 Humble Desktop in Docker (RViz for the Buggy)

Runs a ROS 2 Humble desktop with RViz in Docker on your Mac. You open it in Microsoft **Windows App** like a remote PC, so the Pi only runs the robot and Nav2.

```text
Mac                                              Raspberry Pi
┌──────────────────────────────────┐             ┌───────────────────────────┐
│ Windows App ──RDP localhost:3390 │             │                           │
│        │                         │             │                           │
│ ┌──────▼───────────────────────┐ │  TCP 7447   │ zenoh-bridge-ros2dds      │
│ │ container "buggy-rviz"       │─┼────────────►│   └─ CycloneDDS: Nav2,    │
│ │ XFCE + RViz + zenoh bridge   │ │             │      robot, lidar, map    │
│ └──────────────────────────────┘ │             │                           │
└──────────────────────────────────┘             └───────────────────────────┘
```

- **Image**: `buggy-rviz:humble`, built from `osrf/ros:humble-desktop` (official ROS 2 Humble, Ubuntu 22.04) plus XFCE, xrdp, CycloneDDS, the Nav2 RViz plugins and `zenoh-bridge-ros2dds` 1.10.2. Works on Apple Silicon and Intel Macs.
- **Why a bridge**: Docker puts containers behind NAT, which blocks the UDP traffic ROS 2 (DDS) needs. The Zenoh bridge carries every topic, TF, service and action over one outgoing TCP connection to the Pi.

## Requirements

- Docker Desktop for Mac, running
- Microsoft Windows App (the Remote Desktop client)
- On the Pi: `zenoh-bridge-ros2dds` 1.10.2 in `~/.local/bin` (already installed); `navigation.launch.py` starts it

## Setup (once, on the Mac)

Copy this folder from the Pi:

```bash
ssh qtpi@qtpi.local "tar -C ros2_ws/src/Autonomous_Vehicle/tools -chf - docker_rviz" | tar -xf -
```

If the Pi's IPv4 address is not `172.20.10.5`, edit `PI_ADDR` in `docker_rviz/.env` (`hostname -I` on the Pi shows it).

## Run

1. On the Pi: `ros2 launch robot_bringup navigation.launch.py`
2. On the Mac: `./docker_rviz/start.sh`. It opens an SSH tunnel to the Pi (asks for the Pi's password), then starts the container. The first run builds the image (10–20 minutes, about 4 GB); later starts take seconds.
   The container reaches the Pi through that tunnel (`host.docker.internal:7447`), because macOS can block local IPv4 connections. If the Mac sleeps, run `start.sh` again to reopen the tunnel.
3. In Windows App: **+ → Add PC**, PC name `localhost:3390`, user `ros`, password `ros`. RViz opens after login.

4. In RViz, set **2D Pose Estimate** first. The map appears right away; the robot, laser scan and costmaps appear once AMCL has a pose.

Stop everything with `./docker_rviz/stop.sh`.

Note: `ros2 topic list` inside the container also lists the topics RViz subscribes to, so it shows `/map`, `/scan` and others even when the Pi is not connected. Check the connection with `docker compose logs --tail 20` instead.

## Troubleshooting

- **`ssh: connect to host ... Interrupted system call`**: macOS is blocking Terminal from the local network. Use `qtpi.local` as above, or allow it in System Settings → Privacy & Security → Local Network → Terminal.
- **RViz shows no topics**: run `docker compose logs -f` in `docker_rviz`. "cannot reach 172.20.10.5:7447" means navigation isn't running on the Pi, `PI_ADDR` is wrong, or macOS is blocking Docker: allow **Docker** under Privacy & Security → Local Network.
- **Black screen after login**: disconnect and connect again; if it persists, `docker compose restart`.
- **Map missing**: wait a few seconds after RViz opens; `/map` is sent once and the bridge replays it to late joiners.
