# nav_dashboard

Web dashboard for the buggy's Nav2 stack, served from the Pi.
Built as part of the main workspace (`colcon build` from the workspace root).

## Run (with Nav2 or SLAM already running)

    src/nav_dashboard/run_dashboard.sh                 # http://<pi-ip>:8080/
    src/nav_dashboard/run_dashboard.sh allow_control:=false   # view-only
    src/nav_dashboard/run_dashboard.sh port:=9000

Open the printed URL from any device on the same network.
URL options: `#follow&zoom=60` (zoom = pixels per metre).

## Page

- Map, global costmap, local costmap, planned path, lookahead point, laser scan,
  robot footprint + heading, trail, goal. Layers can be toggled.
- **Nav goal** (G): press at the target, drag toward the final heading, release.
- **Set pose** (P): same gesture, publishes /initialpose for AMCL.
- **Cancel**: cancels all navigate_to_pose goals.
- Drag to pan, wheel/pinch to zoom, F = follow robot, Esc = back to pan.

## Topics (all overridable as ROS parameters)

/map, /global_costmap/costmap(+_updates), /local_costmap/costmap(+_updates),
/plan, /scan, /global_costmap/published_footprint, /lookahead_point, /odom,
/goal_pose, TF map->base_link, navigate_to_pose action. Publishes /initialpose.

## Test without the robot

    export ROS_DOMAIN_ID=77      # keep away from the real robot
    ros2 run nav_dashboard fake_nav &     # simulated robot + Nav2 stand-in
    ros2 run nav_dashboard dashboard

fake_nav refuses to start if real Nav2 nodes or /scan, /odom, /map, /cmd_vel
publishers are visible.

## Rebuild after edits

    cd <workspace> && source /opt/ros/humble/setup.bash && colcon build --packages-select nav_dashboard
