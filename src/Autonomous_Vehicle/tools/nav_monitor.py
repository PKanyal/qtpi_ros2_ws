#!/usr/bin/env python3
"""Lightweight run monitor for the buggy (started by nav_record.sh).

Every second it appends one row to <run>/monitor.csv with topic rates,
scan / TF delay and the AMCL pose + uncertainty, and writes notable moments
to <run>/events.log: initial poses, goals, AMCL jumps, topics going silent,
and "cmd_vel says move but odometry does not move".

Usage: nav_monitor.py <run_dir>
"""

import csv
import math
import os
import sys
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64MultiArray
from tf2_ros import Buffer, TransformListener

STALE_AFTER = 2.0  # s without a message (after having had some) = stale
JUMP_DIST = 0.5    # m between consecutive AMCL poses
JUMP_YAW = 0.5     # rad


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Monitor(Node):

    TOPICS = ("scan", "odom", "enc", "cmd", "amcl")

    def __init__(self, run_dir):
        super().__init__("nav_record_monitor")
        self.events = open(os.path.join(run_dir, "events.log"), "a",
                           buffering=1)
        self.csv_file = open(os.path.join(run_dir, "monitor.csv"), "w",
                             newline="", buffering=1)
        self.cols = [
            "t", "scan_hz", "scan_age_ms", "scan_valid_pct",
            "odom_hz", "odom_x", "odom_y", "odom_yaw", "odom_speed",
            "enc_hz", "enc_l", "enc_r", "cmd_hz", "cmd_v", "cmd_w",
            "amcl_hz", "amcl_x", "amcl_y", "amcl_yaw",
            "amcl_std_xy", "amcl_std_yaw",
            "tf_map_base_age_ms", "tf_odom_base_age_ms",
        ]
        self.csv = csv.DictWriter(self.csv_file, fieldnames=self.cols)
        self.csv.writeheader()

        self.count = {k: 0 for k in self.TOPICS}
        self.last_rx = {k: None for k in self.TOPICS}
        self.stale = {k: False for k in self.TOPICS}
        self.scan_age = []
        self.scan_valid = []
        self.odom = None
        self.odom_speed = 0.0
        self.enc = None
        self.cmd = (0.0, 0.0)
        self.amcl = None
        self.amcl_std = (None, None)
        self.last_initialpose = 0.0
        self.high_std = False
        self.cmd_moving_since = None
        self.stuck_reported = False
        self.odom_at_cmd_start = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Int64MultiArray, "/encoder_counts",
                                 self.on_enc, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_amcl, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self.on_initialpose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_timer(1.0, self.tick)
        self.event("monitor started")

    # ---------------------------------------------------------------- helpers
    def event(self, text):
        self.events.write(f"{time.time():.3f} {text}\n")

    def rx(self, key):
        self.count[key] += 1
        self.last_rx[key] = time.time()
        if self.stale[key]:
            self.stale[key] = False
            self.event(f"TOPIC_BACK {key}")

    # -------------------------------------------------------------- callbacks
    def on_scan(self, msg):
        self.rx("scan")
        self.scan_age.append((time.time() - stamp_s(msg.header.stamp)) * 1e3)
        if msg.ranges:
            ok = sum(1 for r in msg.ranges
                     if math.isfinite(r) and msg.range_min <= r <= msg.range_max)
            self.scan_valid.append(100.0 * ok / len(msg.ranges))

    def on_odom(self, msg):
        self.rx("odom")
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.odom_speed = math.hypot(msg.twist.twist.linear.x,
                                     msg.twist.twist.linear.y)

    def on_enc(self, msg):
        self.rx("enc")
        if len(msg.data) >= 2:
            self.enc = (msg.data[0], msg.data[1])

    def on_cmd(self, msg):
        self.rx("cmd")
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_amcl(self, msg):
        self.rx("amcl")
        p = msg.pose.pose
        new = (p.position.x, p.position.y, yaw_of(p.orientation))
        c = msg.pose.covariance
        std_xy = math.sqrt(max(c[0], c[7], 0.0))
        std_yaw = math.sqrt(max(c[35], 0.0))
        if self.amcl and time.time() - self.last_initialpose > 3.0:
            d = math.hypot(new[0] - self.amcl[0], new[1] - self.amcl[1])
            dyaw = abs(wrap(new[2] - self.amcl[2]))
            if d > JUMP_DIST or dyaw > JUMP_YAW:
                self.event(
                    f"AMCL_JUMP {d:.2f} m {math.degrees(dyaw):.0f} deg "
                    f"({self.amcl[0]:.2f},{self.amcl[1]:.2f}) -> "
                    f"({new[0]:.2f},{new[1]:.2f}) std_xy={std_xy:.2f}")
        if std_xy > 0.5 and not self.high_std:
            self.high_std = True
            self.event(f"AMCL_UNCERTAIN std_xy={std_xy:.2f} m "
                       f"std_yaw={math.degrees(std_yaw):.0f} deg")
        elif std_xy < 0.3 and self.high_std:
            self.high_std = False
            self.event(f"AMCL_CONFIDENT std_xy={std_xy:.2f} m")
        self.amcl = new
        self.amcl_std = (std_xy, std_yaw)

    def on_initialpose(self, msg):
        self.last_initialpose = time.time()
        p = msg.pose.pose
        lag = (time.time() - stamp_s(msg.header.stamp)) * 1e3
        self.event(f"INITIAL_POSE ({p.position.x:.2f},{p.position.y:.2f}) "
                   f"yaw={math.degrees(yaw_of(p.orientation)):.0f} deg "
                   f"stamp_lag={lag:.0f} ms (sender clock vs Pi)")

    def on_goal(self, msg):
        p = msg.pose
        self.event(f"GOAL ({p.position.x:.2f},{p.position.y:.2f}) "
                   f"yaw={math.degrees(yaw_of(p.orientation)):.0f} deg")

    # ------------------------------------------------------------------ tick
    def tf_age(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            return round((time.time() - stamp_s(tf.header.stamp)) * 1e3)
        except Exception:  # noqa: BLE001 - no TF yet is normal
            return ""

    def check_stuck(self, now):
        moving_cmd = abs(self.cmd[0]) > 0.05 or abs(self.cmd[1]) > 0.2
        fresh_cmd = (self.last_rx["cmd"] is not None
                     and now - self.last_rx["cmd"] < 1.0)
        if not (moving_cmd and fresh_cmd and self.odom):
            self.cmd_moving_since = None
            self.stuck_reported = False
            return
        if self.cmd_moving_since is None:
            self.cmd_moving_since = now
            self.odom_at_cmd_start = self.odom
            return
        if now - self.cmd_moving_since > 3.0 and not self.stuck_reported:
            o0, o1 = self.odom_at_cmd_start, self.odom
            moved = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
            turned = abs(wrap(o1[2] - o0[2]))
            if moved < 0.02 and turned < 0.05:
                self.stuck_reported = True
                self.event(
                    f"NOT_MOVING cmd_vel v={self.cmd[0]:.2f} "
                    f"w={self.cmd[1]:.2f} for 3 s but odom did not change "
                    "(motors/encoders/Arduino?)")

    def tick(self):
        now = time.time()
        for k in self.TOPICS:
            last = self.last_rx[k]
            if last is not None and not self.stale[k] and now - last > STALE_AFTER:
                self.stale[k] = True
                self.event(f"TOPIC_STALE {k} (no message for {STALE_AFTER:.0f} s)")
        self.check_stuck(now)

        def r(v, n=3):
            return "" if v is None else round(v, n)

        row = {
            "t": round(now, 3),
            "scan_hz": self.count["scan"],
            "scan_age_ms": r(sum(self.scan_age) / len(self.scan_age), 1)
            if self.scan_age else "",
            "scan_valid_pct": r(sum(self.scan_valid) / len(self.scan_valid), 1)
            if self.scan_valid else "",
            "odom_hz": self.count["odom"],
            "odom_x": r(self.odom and self.odom[0]),
            "odom_y": r(self.odom and self.odom[1]),
            "odom_yaw": r(self.odom and self.odom[2]),
            "odom_speed": r(self.odom_speed),
            "enc_hz": self.count["enc"],
            "enc_l": self.enc[0] if self.enc else "",
            "enc_r": self.enc[1] if self.enc else "",
            "cmd_hz": self.count["cmd"],
            "cmd_v": r(self.cmd[0]),
            "cmd_w": r(self.cmd[1]),
            "amcl_hz": self.count["amcl"],
            "amcl_x": r(self.amcl and self.amcl[0]),
            "amcl_y": r(self.amcl and self.amcl[1]),
            "amcl_yaw": r(self.amcl and self.amcl[2]),
            "amcl_std_xy": r(self.amcl_std[0]),
            "amcl_std_yaw": r(self.amcl_std[1]),
            "tf_map_base_age_ms": self.tf_age("map", "base_link"),
            "tf_odom_base_age_ms": self.tf_age("odom", "base_link"),
        }
        self.csv.writerow(row)
        self.count = {k: 0 for k in self.TOPICS}
        self.scan_age.clear()
        self.scan_valid.clear()


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    rclpy.init()
    node = Monitor(run_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.event("monitor stopped")
        node.events.close()
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
