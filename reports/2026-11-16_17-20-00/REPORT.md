# Run report 2026-11-16_17-20-00

- Start: 2026-11-16 17:20:00 IST (rebuilt from ~/.ros/log; not recorded with navrec)  End: 2026-11-16 17:24:31 (launch still running)  Duration: 4 min 31 s
- Command: `ros2 launch robot_bringup navigation.launch.py`
- Ended by: still running (launch exit code ?)
- Network: ?, DDS peers: ?, SSH from ?
- Code: git ?

## Detected problems (most serious first)

- 🔴 `python3-1` died WHILE RUNNING at 17:20:07 (exit code 1)
- 🔴 /dev/ttyACM0 (Arduino) was missing when the run started: motors and odometry cannot work.
- 🔴 No /encoder_counts messages seen by the monitor at all.
- 🔴 Laptop RViz (rviz_2026-11-16_17-20-28) ended with exit code 137: SIGKILL: force-quit, or killed by the out-of-memory killer
- 🔴 Laptop RViz FROZE (blocked at ~0% CPU from 17:21:07) and was force-quit.
- 🟡 1 TF 'extrapolation' warnings (late TF / clock offset between machines).
- 🟡 QoS mismatch on /particle_cloud: RViz shows nothing for it (set Reliability to match).

## Timeline (first occurrence of each warning type, events)

```
17:20:07  LAUNCH python3-1: process has died [pid 5690, exit code 1, cmd 'python3 /home/qtpi/ros2_pk/src/Autonomous_Vehicle/scripts/motor_serial.py'].
17:20:07  WARN [amcl] AMCL cannot publish a pose or update the transform. Please set the initial pose...
17:20:28  LAPTOP rviz started
17:20:29  WARN [amcl] New subscription discovered on topic '/particle_cloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABI
17:21:01  WARN [amcl] Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time 1794829861.045816 but the latest data is
17:21:24  LAPTOP rviz exited code 137 SIGKILL: force-quit, or killed by the out-of-memory killer
17:23:05  EVENT monitor started
17:23:21  EVENT monitor stopped
```

## Warnings / errors by node (count, normalized message)

**amcl**

```
     26 WARN  AMCL cannot publish a pose or update the transform. Please set the initial pose...
      1 WARN  New subscription discovered on topic '/particle_cloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
      1 WARN  Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time N but the latest data is at time N, when looking up transfor
```

## Topic health while running (from monitor.csv, 1 s windows)

| signal | median | min | 90th pct |
|---|---|---|---|
| scan_hz | 12.00 | 11.00 | 12.00 |
| odom_hz | 20.00 | 20.00 | 20.00 |
| enc_hz | 0.00 | 0.00 | 0.00 |
| cmd_hz | 0.00 | 0.00 | 0.00 |
| amcl_hz | 0.00 | 0.00 | 0.00 |
| scan_age_ms | 85.00 | 81.10 | 85.90 |
| scan_valid_pct | 94.95 | 94.80 | 95.20 |
| tf_map_base_age_ms | 24.00 | 23.00 | 25.00 |
| tf_odom_base_age_ms | 25.00 | 23.00 | 25.00 |

## Pi system

- Temperature: max 67.7 °C, median 67.7 °C
- CPU clock: min 1800 MHz, max 1800 MHz
- Load (1 min): median 3.21, max 3.21
- Avg CPU% of busiest processes: sllidar_node 68%, component_conta 63%, odom_node 10%
- Firmware throttle flags: start None, end None

## Laptop RViz: rviz_2026-11-16_17-20-28

- start: 2026-11-16 17:20:28 IST
- end: 2026-11-16 17:21:24 IST
- exit_code: 137
- exit_meaning: SIGKILL: force-quit, or killed by the out-of-memory killer
- laptop_ip: 172.20.10.13
- pi: qtpi@172.20.10.5
- rviz_display_server: x11 DISPLAY=:1 WAYLAND_DISPLAY=
- gpu: Advanced Micro Devices, Inc. [AMD/ATI] Rembrandt (rev 0a)
- RViz RSS: start 5 MB, max 202 MB; CPU median 7%, max 19%
- Last samples before exit:
```
t=1794829869, state=S, cpu_pct=1, rss_mb=202, threads=28, ping_ms=8.48
t=1794829871, state=S, cpu_pct=1, rss_mb=202, threads=28, ping_ms=33.4
t=1794829873, state=S, cpu_pct=0, rss_mb=202, threads=28, ping_ms=3.71
t=1794829875, state=S, cpu_pct=0, rss_mb=202, threads=28, ping_ms=6.35
t=1794829877, state=S, cpu_pct=1, rss_mb=202, threads=28, ping_ms=6.84
t=1794829879, state=S, cpu_pct=0, rss_mb=202, threads=28, ping_ms=4.96
t=1794829881, state=S, cpu_pct=1, rss_mb=202, threads=28, ping_ms=10.0
t=1794829883, state=S, cpu_pct=0, rss_mb=202, threads=28, ping_ms=8.97
```
- RViz warnings/errors (deduplicated):
```
     1 [rvizN]: Vertex Program:rviz/glslN/indexed_Nbit_image.vert Fragment Program:rviz/glslN/indexed_Nbit_image.frag GLSL link result : 
     1 [rvizN]: Lookup would require extrapolation into the future.  Requested time N but the latest data is at time N, when looking up transform from frame [laser] to frame [ma
```
- RViz console tail:
```
[INFO] [1794829844.325992430] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829843.273 for reason 'discarding message because the queue is full'
[INFO] [1794829844.421915177] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829841.764 for reason 'discarding message because the queue is full'
[INFO] [1794829845.926095919] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829834.803 for reason 'discarding message because the queue is full'
[INFO] [1794829846.853440585] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829845.747 for reason 'discarding message because the queue is full'
[INFO] [1794829847.174367610] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829844.514 for reason 'discarding message because the queue is full'
[INFO] [1794829849.413676709] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829838.053 for reason 'discarding message because the queue is full'
[INFO] [1794829849.445834771] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829848.394 for reason 'discarding message because the queue is full'
[INFO] [1794829849.926027896] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829847.264 for reason 'discarding message because the queue is full'
[INFO] [1794829852.005894132] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829850.949 for reason 'discarding message because the queue is full'
[INFO] [1794829852.677927154] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829850.014 for reason 'discarding message because the queue is full'
[INFO] [1794829852.933653014] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829841.304 for reason 'discarding message because the queue is full'
[INFO] [1794829854.566205733] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829853.510 for reason 'discarding message because the queue is full'
[INFO] [1794829855.429873234] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829852.764 for reason 'discarding message because the queue is full'
[INFO] [1794829856.422261666] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829844.553 for reason 'discarding message because the queue is full'
[INFO] [1794829857.126128322] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829856.072 for reason 'discarding message because the queue is full'
[INFO] [1794829858.150063648] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829855.515 for reason 'discarding message because the queue is full'
[INFO] [1794829859.685756940] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829858.638 for reason 'discarding message because the queue is full'
[INFO] [1794829859.910613024] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829848.303 for reason 'discarding message because the queue is full'
[INFO] [1794829860.678005307] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829858.014 for reason 'discarding message because the queue is full'
[INFO] [1794829861.113299442] [rviz2]: Setting estimate pose: Frame:map, Position(-0.650509, -6.78335, 0), Orientation(0, 0, 0.963102, 0.269138) = Angle: 2.5966
[INFO] [1794829862.183602165] [rviz2]: Trying to create a map of size 60 x 60 using 1 swatches
[INFO] [1794829862.278001376] [rviz]: Message Filter dropping message: frame 'laser' at time 1794829861.192 for reason 'discarding message because the queue is full'
[ERROR] [1794829862.343675429] [rviz2]: Lookup would require extrapolation into the future.  Requested time 1794829862.214789 but the latest data is at time 1794829862.214251, when looking up transform from frame [laser] to frame [map]
[INFO] [1794829863.253427038] [rviz]: Message Filter dropping message: frame 'odom' at time 1794829854.053 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[INFO] [1794829863.657541761] [rviz2]: Trying to create a map of size 778 x 597 using 1 swatches
```

## Console tail (last 40 lines)

```

```
