"""nav_dashboard: live Nav2 web dashboard served from the robot.

One ROS 2 node that subscribes to the map, costmaps, global plan, laser scan,
TF and the navigate_to_pose action topics, and serves a web page plus a
WebSocket on http://<robot>:8080/ . From the page you can send Nav2 goals,
set the AMCL initial pose and cancel navigation.

Only the Python standard library + numpy are used for the web side, so there
is nothing to pip install. Nothing in the navigation stack is modified: this
node only subscribes, sends NavigateToPose goals and publishes /initialpose,
exactly like RViz does.
"""

import asyncio
import collections
import json
import math
import os
import queue
import socket
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (PointStamped, PolygonStamped, PoseStamped,
                               PoseWithCovarianceStamped)
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from .ws_server import WebServer

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "navigating",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING}
TERMINAL_LEVEL = {GoalStatus.STATUS_SUCCEEDED: "success",
                  GoalStatus.STATUS_CANCELED: "warn",
                  GoalStatus.STATUS_ABORTED: "error"}
GRID_KEYS = ("map", "global_costmap", "local_costmap")


# --------------------------------------------------------------------------
# helpers

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose b (x, y, yaw) expressed in frame a -> pose in a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def to_np(data, dtype):
    try:
        return np.frombuffer(data, dtype=dtype)
    except (TypeError, ValueError):
        return np.array(data, dtype=dtype)


def transform_points(T, xy):
    c, s = math.cos(T[2]), math.sin(T[2])
    out = np.empty_like(xy)
    out[:, 0] = T[0] + c * xy[:, 0] - s * xy[:, 1]
    out[:, 1] = T[1] + s * xy[:, 0] + c * xy[:, 1]
    return out


def pack_binary(header, payload):
    """[u32 LE header length][JSON header, padded to 4 bytes][payload]."""
    h = json.dumps(header, separators=(",", ":")).encode()
    h += b" " * ((-(4 + len(h))) % 4)
    return struct.pack("<I", len(h)) + h + payload


def lan_ips():
    ips = []
    try:  # UDP connect sends no packets; it only picks the outgoing interface
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return ips


# --------------------------------------------------------------------------
# shared state between the ROS thread and the web (asyncio) thread

class GridStore:
    """Latest occupancy grid + a log of changed rectangles (for patches)."""

    LOG_LEN = 256

    def __init__(self, key):
        self.key = key
        self.lock = threading.Lock()
        self.meta = None
        self.data = None
        self.seq = 0
        self.full_seq = 0
        self.log = collections.deque(maxlen=self.LOG_LEN)

    def set_full(self, meta, arr):
        with self.lock:
            self.meta = meta
            self.data = arr
            self.seq += 1
            self.full_seq = self.seq
            self.log.clear()

    def apply_update(self, x, y, w, h, arr):
        with self.lock:
            if self.data is None or arr.size != w * h:
                return False
            H, W = self.data.shape
            if x < 0 or y < 0 or x + w > W or y + h > H:
                return False
            self.data[y:y + h, x:x + w] = arr.reshape(h, w)
            self.seq += 1
            self.log.append((self.seq, (x, y, w, h)))
            return True

    def build(self, have_seq):
        """Changes since have_seq -> (seq, is_full, header, raw bytes) or None."""
        with self.lock:
            if self.data is None or self.seq == have_seq:
                return None
            H, W = self.data.shape
            full = (have_seq < self.full_seq or not self.log
                    or self.log[0][0] > have_seq + 1)
            if full:
                x0, y0, x1, y1 = 0, 0, W, H
            else:
                rects = [r for s, r in self.log if s > have_seq]
                x0 = min(r[0] for r in rects)
                y0 = min(r[1] for r in rects)
                x1 = max(r[0] + r[2] for r in rects)
                y1 = max(r[1] + r[3] for r in rects)
            raw = self.data[y0:y1, x0:x1].tobytes()
            header = dict(self.meta, type="grid", key=self.key, full=full,
                          x=x0, y=y0, w=x1 - x0, h=y1 - y0, seq=self.seq)
            return self.seq, full, header, raw


class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.items = {}  # key -> (version, is_binary, payload)
        self.grids = {}
        self.events = collections.deque(maxlen=50)
        self.event_seq = 0

    def put(self, key, payload, binary=False):
        with self.lock:
            version = self.items[key][0] + 1 if key in self.items else 1
            self.items[key] = (version, binary, payload)

    def snapshot(self):
        with self.lock:
            return list(self.items.items())

    def event(self, level, text):
        with self.lock:
            self.event_seq += 1
            self.events.append((self.event_seq, json.dumps(
                {"type": "event", "level": level, "text": text})))

    def events_since(self, seq):
        with self.lock:
            return [e for e in self.events if e[0] > seq]

    def last_event_seq(self):
        with self.lock:
            return self.event_seq


# --------------------------------------------------------------------------
# ROS side

class DashboardNode(Node):
    def __init__(self, hub):
        super().__init__("nav_dashboard")
        self.hub = hub

        def P(name, default):
            return self.declare_parameter(name, default).value

        self.host = P("host", "0.0.0.0")
        self.port = int(P("port", 8080))
        self.web_dir = P("web_dir", "")
        self.global_frame = P("global_frame", "map")
        self.base_frame = P("base_frame", "base_link")
        self.allow_control = bool(P("allow_control", True))
        self.state_rate = float(P("state_rate", 10.0))
        self.scan_rate = float(P("scan_rate", 5.0))
        costmap_rate = float(P("costmap_rate", 1.0))
        self.grid_intervals = {
            "map": max(2.0, 1.0 / costmap_rate),
            "global_costmap": 1.0 / costmap_rate,
            "local_costmap": min(0.5, 1.0 / costmap_rate),
        }
        self.topics = {
            "map": P("map_topic", "/map"),
            "global_costmap": P("global_costmap_topic", "/global_costmap/costmap"),
            "local_costmap": P("local_costmap_topic", "/local_costmap/costmap"),
            "plan": P("plan_topic", "/plan"),
            "scan": P("scan_topic", "/scan"),
            "footprint": P("footprint_topic", "/global_costmap/published_footprint"),
            "lookahead": P("lookahead_topic", "/lookahead_point"),
            "odom": P("odom_topic", "/odom"),
            "goal": P("goal_topic", "/goal_pose"),
            "initialpose": P("initialpose_topic", "/initialpose"),
            "nav_action": P("nav_action", "/navigate_to_pose").rstrip("/"),
        }
        for k in GRID_KEYS:
            hub.grids[k] = GridStore(k)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_seen = {}
        self.pending = {}
        self.last_scan_time = 0.0
        self.odom = None
        self.goal = None
        self.footprint_sent = None
        self.nav_status = None
        self.nav_feedback = None
        self.nav_feedback_time = 0.0
        self._last_status_key = None
        self._grid_subs = {}

        t = self.topics
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Path, t["plan"], lambda m: self._stash("plan", m), be)
        self.create_subscription(LaserScan, t["scan"], lambda m: self._stash("scan", m),
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, t["footprint"],
                                 lambda m: self._stash("footprint", m), be)
        self.create_subscription(PointStamped, t["lookahead"],
                                 lambda m: self._stash("lookahead", m), be)
        self.create_subscription(Odometry, t["odom"], self._on_odom, be)
        self.create_subscription(PoseStamped, t["goal"], self._on_goal_topic, be)

        action = t["nav_action"]
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, action + "/_action/status",
                                 self._on_status, status_qos)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                                 action + "/_action/feedback", self._on_feedback, be)
        self.nav_client = ActionClient(self, NavigateToPose, action)
        self.cancel_client = self.create_client(CancelGoal, action + "/_action/cancel_goal")
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, t["initialpose"], 10)

        self.commands = queue.Queue()
        self.create_timer(1.0, self._ensure_grid_subscriptions)
        self.create_timer(0.05, self._process_commands)
        self.create_timer(1.0 / self.state_rate, self._tick)
        self._ensure_grid_subscriptions()

    # ---- subscriptions ---------------------------------------------------

    def _ensure_grid_subscriptions(self):
        """Subscribe to each grid once a publisher exists, matching its QoS.

        map_server, slam_toolbox and Nav2 costmaps publish TRANSIENT_LOCAL
        (latched); a latched subscription would be incompatible with a
        VOLATILE publisher, so we look before subscribing.
        """
        for key in GRID_KEYS:
            if key in self._grid_subs:
                continue
            topic = self.topics[key]
            infos = self.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            latched = all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                          for i in infos)
            reliable = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                           for i in infos)
            rel = ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
            qos = QoSProfile(depth=1, reliability=rel,
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                                         else DurabilityPolicy.VOLATILE))
            sub = self.create_subscription(
                OccupancyGrid, topic, lambda m, k=key: self._on_grid(k, m), qos)
            upd = self.create_subscription(
                OccupancyGridUpdate, topic + "_updates",
                lambda m, k=key: self._on_grid_update(k, m),
                QoSProfile(depth=10, reliability=rel))
            self._grid_subs[key] = (sub, upd)
            self.get_logger().info(
                f"subscribed to {topic} ({'latched' if latched else 'volatile'})")

    def _stash(self, key, msg):
        self.pending[key] = msg
        self.last_seen[key] = time.monotonic()

    def _on_grid(self, key, msg):
        w, h = msg.info.width, msg.info.height
        arr = to_np(msg.data, np.int8)
        if arr.size != w * h or w == 0 or h == 0:
            self.get_logger().warning(f"{key}: bad grid size", throttle_duration_sec=10.0)
            return
        o = msg.info.origin
        meta = {
            "frame": msg.header.frame_id.lstrip("/") or self.global_frame,
            "res": msg.info.resolution, "width": w, "height": h,
            "ox": o.position.x, "oy": o.position.y, "oyaw": yaw_of(o.orientation),
        }
        self.hub.grids[key].set_full(meta, arr.reshape(h, w).copy())
        self.last_seen[key] = time.monotonic()

    def _on_grid_update(self, key, msg):
        arr = to_np(msg.data, np.int8)
        if self.hub.grids[key].apply_update(msg.x, msg.y, msg.width, msg.height, arr):
            self.last_seen[key] = time.monotonic()

    def _on_odom(self, msg):
        self.odom = msg
        self.last_seen["odom"] = time.monotonic()

    def _on_goal_topic(self, msg):
        T = self._tf2d(msg.header.frame_id)
        if T is None:
            return
        p = msg.pose
        g = compose(T, (p.position.x, p.position.y, yaw_of(p.orientation)))
        self.goal = {"x": round(g[0], 3), "y": round(g[1], 3), "yaw": round(g[2], 4)}

    def _on_status(self, msg):
        if not msg.status_list:
            return
        latest = max(msg.status_list,
                     key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        self.nav_status = latest.status
        key = (bytes(latest.goal_info.goal_id.uuid), latest.status)
        first = self._last_status_key is None
        if key != self._last_status_key:
            self._last_status_key = key
            if not first and latest.status in TERMINAL_LEVEL:
                self.hub.event(TERMINAL_LEVEL[latest.status],
                               f"Navigation {STATUS_NAMES[latest.status]}")

    def _on_feedback(self, msg):
        fb = msg.feedback

        def secs(d):
            return d.sec + d.nanosec * 1e-9

        self.nav_feedback = {
            "distance": round(float(fb.distance_remaining), 2),
            "eta": round(secs(fb.estimated_time_remaining), 1),
            "time": round(secs(fb.navigation_time), 1),
            "recoveries": int(fb.number_of_recoveries),
        }
        self.nav_feedback_time = time.monotonic()

    # ---- periodic processing -------------------------------------------

    def _tf2d(self, frame, target=None):
        target = target or self.global_frame
        frame = (frame or "").lstrip("/")
        if not frame or frame == target:
            return (0.0, 0.0, 0.0)
        try:
            t = self.tf_buffer.lookup_transform(target, frame, Time())
        except tf2_ros.TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))

    def _tick(self):
        now = time.monotonic()
        for key in ("plan", "footprint", "lookahead"):
            msg = self.pending.pop(key, None)
            if msg is not None:
                try:
                    getattr(self, "_process_" + key)(msg)
                except Exception as exc:
                    self.get_logger().warning(f"{key}: {exc!r}", throttle_duration_sec=10.0)
        if "scan" in self.pending and now - self.last_scan_time >= 1.0 / self.scan_rate:
            self.last_scan_time = now
            try:
                self._process_scan(self.pending.pop("scan"))
            except Exception as exc:
                self.get_logger().warning(f"scan: {exc!r}", throttle_duration_sec=10.0)
        self._publish_state(now)

    def _process_plan(self, msg):
        T = self._tf2d(msg.header.frame_id)
        if T is None:
            return
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses],
                      dtype=np.float64).reshape(-1, 2)
        if len(xy):
            xy = transform_points(T, xy)
            if len(xy) > 600:
                xy = xy[np.linspace(0, len(xy) - 1, 600).astype(int)]
        self.hub.put("plan", json.dumps(
            {"type": "path", "key": "plan", "pts": np.round(xy, 3).ravel().tolist()}))

    def _process_footprint(self, msg):
        # Sent in the robot's own frame so the page can draw it at the live pose.
        T = self._tf2d(msg.header.frame_id, target=self.base_frame)
        if T is None or not msg.polygon.points:
            return
        xy = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float64)
        pts = np.round(transform_points(T, xy), 3).ravel().tolist()
        if pts != self.footprint_sent:
            self.footprint_sent = pts
            self.hub.put("footprint", json.dumps({"type": "footprint", "pts": pts}))

    def _process_lookahead(self, msg):
        T = self._tf2d(msg.header.frame_id)
        if T is None:
            return
        x, y, _ = compose(T, (msg.point.x, msg.point.y, 0.0))
        self.hub.put("lookahead", json.dumps(
            {"type": "path", "key": "lookahead", "pts": [round(x, 3), round(y, 3)]}))

    def _process_scan(self, msg):
        T = self._tf2d(msg.header.frame_id)
        if T is None:
            return
        r = to_np(msg.ranges, np.float32)
        a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.01)) & (r <= msg.range_max)
        r, a = r[ok], a[ok]
        step = max(1, -(-len(r) // 720))
        r, a = r[::step], a[::step]
        xy = np.stack([r * np.cos(a), r * np.sin(a)], axis=1).astype(np.float64)
        xy = transform_points(T, xy).astype("<f4") if len(xy) else xy.astype("<f4")
        self.hub.put("scan", pack_binary(
            {"type": "scan", "ox": round(T[0], 3), "oy": round(T[1], 3), "n": len(xy)},
            xy.tobytes()), binary=True)

    def _publish_state(self, now):
        robot = self._tf2d(self.base_frame)
        frames = {}
        for k in GRID_KEYS:
            meta = self.hub.grids[k].meta
            if meta and meta["frame"] not in ("", self.global_frame):
                T = self._tf2d(meta["frame"])
                if T is not None:
                    frames[meta["frame"]] = [round(v, 4) for v in T]
        vel = None
        if self.odom is not None and now - self.last_seen.get("odom", 0) < 2.0:
            tw = self.odom.twist.twist
            vel = {"v": round(tw.linear.x, 3), "w": round(tw.angular.z, 3)}
        fb = self.nav_feedback if now - self.nav_feedback_time < 3.0 else None
        ages = {}
        for key in ("map", "global_costmap", "local_costmap", "scan", "odom", "plan"):
            seen = self.last_seen.get(key)
            ages[key] = round(now - seen, 1) if seen else None
        state = {
            "type": "state",
            "robot": ({"x": round(robot[0], 3), "y": round(robot[1], 3),
                       "yaw": round(robot[2], 4)} if robot else None),
            "frames": frames,
            "vel": vel,
            "nav": {
                "ready": self.nav_client.server_is_ready(),
                "status": STATUS_NAMES.get(self.nav_status),
                "active": self.nav_status in ACTIVE,
                "goal": self.goal,
                "feedback": fb,
            },
            "ages": ages,
        }
        self.hub.put("state", json.dumps(state, separators=(",", ":")))

    # ---- commands from the browser --------------------------------------

    def _process_commands(self):
        while True:
            try:
                cmd = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                self._run_command(cmd)
            except Exception as exc:
                self.hub.event("error", f"{cmd.get('cmd', '?')}: {exc}")

    def _pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def _run_command(self, cmd):
        kind = cmd.get("cmd")
        if not self.allow_control:
            self.hub.event("error", "Control is disabled (allow_control:=false)")
            return

        def num(k):
            v = float(cmd[k])
            if not math.isfinite(v):
                raise ValueError(f"bad {k}")
            return v

        if kind == "goal":
            x, y, yaw = num("x"), num("y"), num("yaw")
            if not self.nav_client.server_is_ready():
                self.hub.event("error", "Nav2 is not running (no navigate_to_pose server)")
                return
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(x, y, yaw)
            self.nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)
            self.goal = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)}
            self.get_logger().info(f"goal from dashboard: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
            self.hub.event("info", f"Goal sent ({x:.2f}, {y:.2f})")
        elif kind == "initialpose":
            x, y, yaw = num("x"), num("y"), num("yaw")
            msg = PoseWithCovarianceStamped()
            ps = self._pose(x, y, yaw)
            msg.header, msg.pose.pose = ps.header, ps.pose
            cov = [0.0] * 36
            cov[0], cov[7], cov[35] = 0.25, 0.25, 0.06853891945200942  # RViz defaults
            msg.pose.covariance = cov
            self.initpose_pub.publish(msg)
            self.get_logger().info(f"initial pose from dashboard: x={x:.2f} y={y:.2f}")
            self.hub.event("info", f"Initial pose set ({x:.2f}, {y:.2f})")
        elif kind == "cancel":
            if not self.cancel_client.service_is_ready():
                self.hub.event("error", "Nav2 cancel service not available")
                return
            # empty request (zero goal id + zero stamp) = cancel all goals
            fut = self.cancel_client.call_async(CancelGoal.Request())
            fut.add_done_callback(self._on_cancel_response)
        else:
            self.hub.event("error", f"unknown command {kind!r}")

    def _on_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as exc:
            self.hub.event("error", f"Goal failed: {exc}")
            return
        if gh.accepted:
            self.hub.event("info", "Goal accepted by Nav2")
        else:
            self.hub.event("error", "Nav2 rejected the goal")

    def _on_cancel_response(self, fut):
        try:
            n = len(fut.result().goals_canceling)
        except Exception as exc:
            self.hub.event("error", f"Cancel failed: {exc}")
            return
        self.hub.event("warn" if n else "info",
                       f"Canceling {n} goal(s)" if n else "Nothing to cancel")


# --------------------------------------------------------------------------
# web side (runs an asyncio loop in a background thread)

class DashboardServer:
    def __init__(self, node, hub, web_dir):
        self.node = node
        self.hub = hub
        self.loop = asyncio.new_event_loop()
        self.server = WebServer(node.host, node.port, web_dir, self._handle_ws,
                                log=lambda s: node.get_logger().warning(s))
        self.full_cache = {}
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        self.ready.wait(5.0)
        if self.error:
            raise self.error

    def stop(self):
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(2.0)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.server.start())
        except OSError as exc:
            self.error = exc
            self.ready.set()
            return
        self.ready.set()
        self.loop.run_forever()

    async def _compress(self, raw):
        if len(raw) > 65536:  # big grids: keep the event loop responsive
            return await self.loop.run_in_executor(None, zlib.compress, raw, 1)
        return zlib.compress(raw, 1)

    async def _handle_ws(self, ws):
        log = self.node.get_logger()
        log.info(f"browser connected: {ws.peer[0] if ws.peer else '?'}")
        client = {"items": {}, "grids": {}, "grid_time": {},
                  "event_seq": self.hub.last_event_seq()}
        reader = asyncio.ensure_future(self._reader(ws))
        try:
            await ws.send_text(json.dumps({
                "type": "hello", "global_frame": self.node.global_frame,
                "base_frame": self.node.base_frame,
                "allow_control": self.node.allow_control,
                "topics": {k: v for k, v in self.node.topics.items()},
            }))
            while not ws.closed and not reader.done():
                await self._push(ws, client)
                await asyncio.sleep(0.05)
        except (ConnectionError, OSError):
            pass
        finally:
            reader.cancel()
            log.info(f"browser disconnected: {ws.peer[0] if ws.peer else '?'}")

    async def _reader(self, ws):
        while True:
            msg = await ws.receive()
            if msg is None:
                return
            if isinstance(msg, str):
                try:
                    data = json.loads(msg)
                except ValueError:
                    continue
                if isinstance(data, dict) and "cmd" in data:
                    self.node.commands.put(data)

    async def _push(self, ws, client):
        for seq, text in self.hub.events_since(client["event_seq"]):
            await ws.send_text(text)
            client["event_seq"] = seq

        now = time.monotonic()
        for key, store in self.hub.grids.items():
            have = client["grids"].get(key, 0)
            if store.seq == have:
                continue
            if now - client["grid_time"].get(key, 0.0) < self.node.grid_intervals[key]:
                continue
            built = store.build(have)
            if not built:
                continue
            seq, full, header, raw = built
            cached = self.full_cache.get(key)
            if full and cached and cached[0] == seq:
                frame = cached[1]
            else:
                frame = pack_binary(header, await self._compress(raw))
                if full:
                    self.full_cache[key] = (seq, frame)
            await ws.send_bytes(frame)
            client["grids"][key] = seq
            client["grid_time"][key] = now

        for key, (version, binary, payload) in self.hub.snapshot():
            if client["items"].get(key) == version:
                continue
            if binary:
                await ws.send_bytes(payload)
            else:
                await ws.send_text(payload)
            client["items"][key] = version


def main(args=None):
    rclpy.init(args=args)
    hub = Hub()
    node = DashboardNode(hub)
    web_dir = node.web_dir or os.path.join(
        get_package_share_directory("nav_dashboard"), "web")
    server = DashboardServer(node, hub, web_dir)
    try:
        server.start()
        urls = ", ".join(f"http://{ip}:{node.port}/" for ip in lan_ips()) or \
            f"http://<robot-ip>:{node.port}/"
        node.get_logger().info(f"dashboard running: {urls}")
        rclpy.spin(node)
    except OSError as exc:
        node.get_logger().error(f"cannot open port {node.port}: {exc}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
