"""fake_nav: a simulated robot + Nav2 stand-in for testing the dashboard.

Loads a saved map and publishes what the real stack would: /map,
/global_costmap/costmap (+ _updates with a moving obstacle), a rolling
/local_costmap/costmap in the odom frame, /scan, /odom, /plan, the footprint,
/lookahead_point and TF map->odom->base_link->laser. It also serves a
navigate_to_pose action that drives the simulated robot in a straight line
(ignoring walls), and listens to /initialpose.

It never talks to hardware, and it refuses to start when real Nav2 nodes or
robot topics are visible. Run it on its own ROS_DOMAIN_ID to be safe, e.g.

    ROS_DOMAIN_ID=77 ros2 run nav_dashboard fake_nav --ros-args -p map:=/path/map.yaml
"""

import array
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import (Point32, PointStamped, PolygonStamped,
                               PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

REAL_NODES = {"bt_navigator", "controller_server", "planner_server", "amcl",
              "slam_toolbox", "map_server", "velocity_smoother"}
REAL_TOPICS = ("/scan", "/odom", "/map", "/cmd_vel")
FOOTPRINT = [(0.16, 0.145), (0.16, -0.145), (-0.13, -0.145), (-0.13, 0.145)]
MAX_V, MAX_W = 0.18, 1.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def invert(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5" or maxval > 255:
        raise ValueError(f"{path}: only 8-bit binary PGM (P5) maps are supported")
    return np.frombuffer(data, np.uint8, w * h, i).reshape(h, w)


def load_map(yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    img = info["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(yaml_path), img)
    pix = read_pgm(img).astype(np.float32)
    p = pix / 255.0 if int(info.get("negate", 0)) else (255.0 - pix) / 255.0
    grid = np.full(pix.shape, -1, np.int8)
    grid[p > float(info.get("occupied_thresh", 0.65))] = 100
    grid[p < float(info.get("free_thresh", 0.25))] = 0
    return np.flipud(grid).copy(), float(info["resolution"]), info["origin"]


def inflate(grid, res, radius=0.35, inscribed=0.17):
    """Cheap Nav2-like inflation (Manhattan rings)."""
    lethal = grid == 100
    out = grid.astype(np.int16)
    reached = lethal.copy()
    frontier = lethal.copy()
    for d in range(1, int(radius / res) + 1):
        grown = frontier.copy()
        grown[1:, :] |= frontier[:-1, :]
        grown[:-1, :] |= frontier[1:, :]
        grown[:, 1:] |= frontier[:, :-1]
        grown[:, :-1] |= frontier[:, 1:]
        new = grown & ~reached
        dist = d * res
        cost = 99 if dist <= inscribed else int(98 * math.exp(-6.0 * (dist - inscribed)))
        out[new] = np.maximum(out[new], cost)
        reached |= new
        frontier = new
    return out.astype(np.int8), reached


def default_map():
    """<workspace>/maps/c205a.yaml, found from where this package is installed
    ($BUGGY_WS overrides; falls back to ~/ros2_ws)."""
    ws = os.environ.get("BUGGY_WS")
    if not ws:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = get_package_share_directory("nav_dashboard")
            while os.path.dirname(path) != path and os.path.basename(path) != "install":
                path = os.path.dirname(path)
            if os.path.basename(path) == "install":
                ws = os.path.dirname(path)
        except Exception:
            pass
    ws = os.path.expanduser(ws or "~/ros2_ws")
    return os.path.join(ws, "maps", "c205a.yaml")


class FakeNav(Node):
    def __init__(self):
        super().__init__("fake_nav")
        map_yaml = self.declare_parameter("map", default_map()).value
        self.lock = threading.Lock()
        t0 = time.monotonic()
        self.static, self.res, origin = load_map(map_yaml)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.base_cost, reached = inflate(self.static, self.res)
        self.cost = self.base_cost.copy()
        self.H, self.W = self.static.shape
        self.get_logger().info(
            f"loaded {map_yaml} ({self.W}x{self.H}) in {time.monotonic() - t0:.1f}s")

        # start in the open free cell closest to the middle of the free space
        cand = np.argwhere((self.static == 0) & ~reached)
        mid = cand.mean(axis=0)
        r, c = cand[np.argmin(((cand - mid) ** 2).sum(axis=1))]
        self.pose = (self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res, 0.0)
        self.v = self.w = 0.0
        self.target = None
        self.active_goal = None
        self.reached = False
        self.map_odom = (0.4, -0.25, 0.08)  # deliberately non-zero to test TF use
        self.blob_region = None
        self.blob_phase = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.gcm_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", latched)
        self.gcm_upd_pub = self.create_publisher(
            OccupancyGridUpdate, "/global_costmap/costmap_updates", latched)
        self.lcm_pub = self.create_publisher(OccupancyGrid, "/local_costmap/costmap", latched)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.plan_pub = self.create_publisher(Path, "/plan", 1)
        self.fp_pub = self.create_publisher(
            PolygonStamped, "/global_costmap/published_footprint", 1)
        self.la_pub = self.create_publisher(PointStamped, "/lookahead_point", 1)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self._tf("base_link", "laser", (0.1, 0.0, math.pi)))

        self.map_pub.publish(self._grid(self.static, "map", self.ox, self.oy))
        self.gcm_pub.publish(self._grid(self.cost, "map", self.ox, self.oy))

        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self._on_initialpose, 10)
        cb = ReentrantCallbackGroup()
        self.action = ActionServer(
            self, NavigateToPose, "navigate_to_pose", execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cb)
        self.create_timer(0.05, self._sim)
        self.create_timer(0.2, self._scan)
        self.create_timer(1.0, self._slow)
        self.get_logger().info(
            f"fake robot at ({self.pose[0]:.2f}, {self.pose[1]:.2f}); ready")

    # ---- message helpers ----
    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _tf(self, parent, child, p):
        t = TransformStamped()
        t.header.stamp = self._stamp()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = p[0], p[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
        return t

    def _grid(self, arr, frame, ox, oy):
        m = OccupancyGrid()
        m.header.frame_id, m.header.stamp = frame, self._stamp()
        m.info.resolution = self.res
        m.info.height, m.info.width = arr.shape
        m.info.origin.position.x, m.info.origin.position.y = ox, oy
        m.info.origin.orientation.w = 1.0
        m.data = array.array("b", np.ascontiguousarray(arr, dtype=np.int8).tobytes())
        return m

    def _pose_stamped(self, p, frame="map"):
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = frame, self._stamp()
        ps.pose.position.x, ps.pose.position.y = p[0], p[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
        return ps

    # ---- simulation ----
    def _sim(self):
        dt = 0.05
        with self.lock:
            x, y, yaw = self.pose
            v = w = 0.0
            if self.target:
                tx, ty, tyaw = self.target
                dist = math.hypot(tx - x, ty - y)
                if dist > 0.05:
                    err = wrap(math.atan2(ty - y, tx - x) - yaw)
                    w = max(-MAX_W, min(MAX_W, 2.0 * err))
                    v = 0.0 if abs(err) > 0.35 else min(MAX_V, dist)
                else:
                    err = wrap(tyaw - yaw)
                    if abs(err) > 0.05:
                        w = max(-MAX_W, min(MAX_W, 2.0 * err))
                    else:
                        self.target, self.reached = None, True
            yaw = wrap(yaw + w * dt)
            x, y = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
            self.pose, self.v, self.w = (x, y, yaw), v, w
            pose = self.pose
        odom_base = compose(invert(self.map_odom), pose)
        self.tf_pub.sendTransform([self._tf("map", "odom", self.map_odom),
                                   self._tf("odom", "base_link", odom_base)])
        o = Odometry()
        o.header.frame_id, o.header.stamp, o.child_frame_id = "odom", self._stamp(), "base_link"
        o.pose.pose = self._pose_stamped(odom_base, "odom").pose
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.odom_pub.publish(o)

    def _scan(self):
        with self.lock:
            pose = self.pose
            occ = self.cost == 100
        lp = compose(pose, (0.1, 0.0, math.pi))
        n, max_r = 360, 8.0
        rel = -math.pi + np.arange(n) * (2 * math.pi / n)
        ang = lp[2] + rel
        steps = np.arange(0.1, max_r, self.res)
        xs = lp[0] + np.cos(ang)[:, None] * steps
        ys = lp[1] + np.sin(ang)[:, None] * steps
        cols = np.floor((xs - self.ox) / self.res).astype(int)
        rows = np.floor((ys - self.oy) / self.res).astype(int)
        inside = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
        hit = np.zeros_like(inside)
        hit[inside] = occ[rows[inside], cols[inside]]
        ranges = np.where(hit.any(axis=1), steps[hit.argmax(axis=1)], np.inf).astype(np.float32)
        ranges += np.random.normal(0, 0.01, n).astype(np.float32)
        m = LaserScan()
        m.header.frame_id, m.header.stamp = "laser", self._stamp()
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / n
        m.angle_max = m.angle_min + (n - 1) * m.angle_increment
        m.range_min, m.range_max = 0.1, max_r
        m.ranges = array.array("f", ranges.tobytes())
        self.scan_pub.publish(m)

    def _slow(self):
        with self.lock:
            pose, target = self.pose, self.target
        self._move_blob(pose)
        self._local_costmap(pose)
        fp = PolygonStamped()
        fp.header.frame_id, fp.header.stamp = "map", self._stamp()
        for px, py in FOOTPRINT:
            wx, wy, _ = compose(pose, (px, py, 0.0))
            fp.polygon.points.append(Point32(x=float(wx), y=float(wy)))
        self.fp_pub.publish(fp)
        if target:
            dist = math.hypot(target[0] - pose[0], target[1] - pose[1])
            n = max(2, int(dist / 0.05))
            path = Path()
            path.header.frame_id, path.header.stamp = "map", self._stamp()
            for i in range(n + 1):
                f = i / n
                path.poses.append(self._pose_stamped(
                    (pose[0] + f * (target[0] - pose[0]), pose[1] + f * (target[1] - pose[1]),
                     target[2])))
            self.plan_pub.publish(path)
            la = PointStamped()
            la.header.frame_id, la.header.stamp = "map", self._stamp()
            f = min(1.0, 0.6 / max(dist, 1e-6))
            la.point.x = pose[0] + f * (target[0] - pose[0])
            la.point.y = pose[1] + f * (target[1] - pose[1])
            self.la_pub.publish(la)

    def _publish_update(self, r0, c0, h, w):
        u = OccupancyGridUpdate()
        u.header.frame_id, u.header.stamp = "map", self._stamp()
        u.x, u.y, u.width, u.height = int(c0), int(r0), int(w), int(h)
        u.data = array.array("b", self.cost[r0:r0 + h, c0:c0 + w].tobytes())
        self.gcm_upd_pub.publish(u)

    def _move_blob(self, pose):
        """A 'person' circling the robot, sent as costmap_updates patches."""
        size, rad = 18, 4
        with self.lock:
            if self.blob_region:
                r0, c0 = self.blob_region
                self.cost[r0:r0 + size, c0:c0 + size] = self.base_cost[r0:r0 + size, c0:c0 + size]
                self._publish_update(r0, c0, size, size)
            self.blob_phase += 0.35
            bx = pose[0] + 1.2 * math.cos(self.blob_phase)
            by = pose[1] + 1.2 * math.sin(self.blob_phase)
            c0 = int((bx - self.ox) / self.res) - size // 2
            r0 = int((by - self.oy) / self.res) - size // 2
            if not (0 <= r0 < self.H - size and 0 <= c0 < self.W - size):
                self.blob_region = None
                return
            yy, xx = np.mgrid[0:size, 0:size] - (size - 1) / 2
            d = np.hypot(yy, xx)
            patch = self.cost[r0:r0 + size, c0:c0 + size].astype(np.int16)
            ring = np.where(d <= rad, 100, np.where(d <= rad + 3, 99, np.where(
                d <= size / 2, 60 * np.exp(-(d - rad - 3) / 2), 0))).astype(np.int16)
            self.cost[r0:r0 + size, c0:c0 + size] = np.maximum(patch, ring).astype(np.int8)
            self.blob_region = (r0, c0)
            self._publish_update(r0, c0, size, size)

    def _local_costmap(self, pose):
        n = int(3.0 / self.res)
        ob = compose(invert(self.map_odom), pose)
        ox = math.floor((ob[0] - 1.5) / self.res) * self.res
        oy = math.floor((ob[1] - 1.5) / self.res) * self.res
        gy, gx = np.mgrid[0:n, 0:n]
        X, Y = ox + (gx + 0.5) * self.res, oy + (gy + 0.5) * self.res
        c, s = math.cos(self.map_odom[2]), math.sin(self.map_odom[2])
        MX = self.map_odom[0] + c * X - s * Y
        MY = self.map_odom[1] + s * X + c * Y
        cols = np.floor((MX - self.ox) / self.res).astype(int)
        rows = np.floor((MY - self.oy) / self.res).astype(int)
        inside = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
        out = np.zeros((n, n), np.int8)
        with self.lock:
            out[inside] = self.cost[rows[inside], cols[inside]]
        out[out < 0] = 0
        self.lcm_pub.publish(self._grid(out, "odom", ox, oy))

    # ---- interfaces ----
    def _on_initialpose(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.get_logger().info(f"initial pose -> ({p.position.x:.2f}, {p.position.y:.2f})")

    def _execute(self, gh):
        p = gh.request.pose.pose
        with self.lock:
            self.target = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.active_goal, self.reached = gh, False
        self.get_logger().info(f"goal -> ({p.position.x:.2f}, {p.position.y:.2f})")
        start = time.monotonic()
        while rclpy.ok():
            with self.lock:
                mine = self.active_goal is gh
                done = mine and self.reached
                pose, target = self.pose, self.target
            if not mine:
                gh.abort()
                return NavigateToPose.Result()
            if gh.is_cancel_requested:
                with self.lock:
                    self.target, self.active_goal = None, None
                gh.canceled()
                self.get_logger().info("goal canceled")
                return NavigateToPose.Result()
            if done:
                with self.lock:
                    self.active_goal = None
                gh.succeed()
                self.get_logger().info("goal reached")
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.current_pose = self._pose_stamped(pose)
            dist = math.hypot(target[0] - pose[0], target[1] - pose[1]) if target else 0.0
            fb.distance_remaining = float(dist)
            el, eta = time.monotonic() - start, dist / MAX_V
            fb.navigation_time = DurationMsg(sec=int(el), nanosec=int((el % 1) * 1e9))
            fb.estimated_time_remaining = DurationMsg(sec=int(eta), nanosec=int((eta % 1) * 1e9))
            gh.publish_feedback(fb)
            time.sleep(0.1)
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    probe = rclpy.create_node("fake_nav_probe")
    time.sleep(2.0)  # let DDS discovery fill in the graph
    clash = sorted(set(probe.get_node_names()) & REAL_NODES)
    busy = [t for t in REAL_TOPICS if probe.get_publishers_info_by_topic(t)]
    probe.destroy_node()
    if clash or busy:
        print("fake_nav: refusing to start, the real robot/Nav2 is visible on this "
              f"ROS domain (nodes: {clash or '-'}, topics: {busy or '-'}).\n"
              "Use a separate ROS_DOMAIN_ID, e.g.  ROS_DOMAIN_ID=77 ros2 run nav_dashboard fake_nav",
              file=sys.stderr)
        rclpy.try_shutdown()
        return 1
    node = FakeNav()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
