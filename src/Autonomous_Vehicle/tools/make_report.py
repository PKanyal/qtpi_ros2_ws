#!/usr/bin/env python3
"""Build REPORT.md for one nav_record.sh run.

Usage:
    make_report.py [run_dir] [--print-summary]

run_dir defaults to <workspace>/reports/latest ($QTPI_REPORTS overrides). Safe to re-run: e.g. after the
laptop uploaded its RViz report, run it again to merge that in.
"""

import csv
import glob
import gzip
import os
import re
import statistics
import sys
import time
from collections import Counter, defaultdict

# <ws>/src/Autonomous_Vehicle/tools/make_report.py -> <ws>/reports
_WS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
REPORTS = os.path.expanduser(os.environ.get("QTPI_REPORTS", os.path.join(_WS, "reports")))

NODE_LINE = re.compile(
    r"\[(WARN|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)")
LAUNCH_LINE = re.compile(r"^(\d+\.\d+) \[(\w+)\] \[([^\]]+)\]: (.*)")
NUM = re.compile(r"-?\d+(\.\d+)?")


def opener(path):
    return gzip.open(path, "rt", errors="replace") if path.endswith(".gz") \
        else open(path, errors="replace")


def read_lines(path):
    try:
        with opener(path) as f:
            return f.read().splitlines()
    except OSError:
        return []


def find(run, name):
    for cand in (os.path.join(run, name), os.path.join(run, name + ".gz")):
        if os.path.exists(cand):
            return cand
    return None


def read_meta(path):
    meta = {}
    for line in read_lines(path):
        if "=" in line:
            k, v = line.split("=", 1)
            meta[k.strip()] = v.strip()
    return meta


def read_csv(path):
    if not path:
        return []
    with opener(path) as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def hms(t):
    return time.strftime("%H:%M:%S", time.localtime(t))


def normalize(msg):
    msg = NUM.sub("N", msg)
    return msg[:170]


def col(rows, key):
    return [x for x in (fnum(r.get(key)) for r in rows) if x is not None]


def pct(values, q):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))]


# --------------------------------------------------------------------------
def parse_ros_logs(run):
    """Return (warnings Counter[(node, level, msg)], first/last ts, launch events)."""
    hist = Counter()
    first_seen = {}
    timeline = []
    files = glob.glob(os.path.join(run, "ros_log", "**", "*.log*"),
                      recursive=True)
    launch_events = []
    for path in files:
        is_launch = os.path.basename(path).startswith("launch.log")
        for line in read_lines(path):
            if is_launch:
                m = LAUNCH_LINE.match(line)
                if m:
                    ts, level, who, msg = m.groups()
                    if ("process has died" in msg or "user interrupted" in msg
                            or "process started" in msg
                            or "escalating" in msg):
                        launch_events.append((float(ts), level, who, msg))
                continue
            m = NODE_LINE.search(line)
            if not m:
                continue
            level, ts, node, msg = m.groups()
            key = (node, level, normalize(msg))
            hist[key] += 1
            if key not in first_seen:
                first_seen[key] = float(ts)
                timeline.append((float(ts), f"{level} [{node}] {msg[:150]}"))
    return hist, first_seen, timeline, launch_events


def console_crashes(run):
    """Tracebacks / fatal native errors that only show on stdout/stderr."""
    lines = read_lines(find(run, "console.log") or "")
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if "Traceback (most recent call last)" in line:
            block = [line]
            j = i + 1
            while j < len(lines) and len(block) < 40:
                block.append(lines[j])
                # the exception line ends the traceback
                if re.search(r"\]\s*\w+(Error|Exception|Interrupt)\b", lines[j]) \
                        or re.match(r"^\s*\w+(Error|Exception):", lines[j]):
                    break
                j += 1
            out.append("\n".join(block))
            i = j
        elif re.search(r"Segmentation fault|terminate called|core dumped|"
                       r"std::bad_alloc|Aborted|what\(\):", line):
            out.append(line)
        i += 1
    return out, lines


def laptop_reports(start, end):
    found = []
    for meta_path in glob.glob(os.path.join(REPORTS, "laptop", "*", "meta.txt")):
        meta = read_meta(meta_path)
        s, e = fnum(meta.get("start_epoch")), fnum(meta.get("end_epoch"))
        if s is None:
            continue
        e = e or s
        if s <= end + 120 and e >= start - 120:
            found.append((os.path.dirname(meta_path), meta))
    return sorted(found, key=lambda x: fnum(x[1]["start_epoch"]))


# --------------------------------------------------------------------------
def build(run):
    meta = read_meta(os.path.join(run, "meta.txt"))
    start = fnum(meta.get("start_epoch")) or 0.0
    end = fnum(meta.get("end_epoch")) or time.time()
    hist, first_seen, timeline, launch_events = parse_ros_logs(run)
    crashes, console = console_crashes(run)
    mon = read_csv(find(run, "monitor.csv"))
    sysrows = read_csv(find(run, "system.csv"))
    events = []
    for line in read_lines(find(run, "events.log") or ""):
        ts, _, text = line.partition(" ")
        if fnum(ts):
            events.append((float(ts), text))
    kernel = [l for l in read_lines(find(run, "kernel.log") or "")
              if re.search(r"USB disconnect|new .*USB device|under-voltage|"
                           r"[Tt]hrottl|oom|Out of memory|cp210x|cdc_acm|"
                           r"ttyACM|ttyUSB|error|WARNING", l, re.I)]

    stop_ts = min([t for t, _, _, m in launch_events if "user interrupted" in m]
                  or [end])
    problems = []  # (severity, text)

    # ---- process deaths
    for ts, level, who, msg in launch_events:
        if "process has died" in msg:
            when = "during shutdown" if ts >= stop_ts else "WHILE RUNNING"
            sev = 1 if when == "WHILE RUNNING" else 3
            code = re.search(r"exit code (-?\d+)", msg)
            problems.append((sev, f"`{who}` died {when} at {hms(ts)} "
                                  f"(exit code {code.group(1) if code else '?'})"))
    for c in crashes:
        problems.append((1, "Crash output in console:\n```\n" + c[-1500:] + "\n```"))

    # ---- Arduino / USB
    serial_err = sum(n for (node, _, m), n in hist.items()
                     if node == "motor_serial" and "Serial" in m)
    if serial_err:
        k = next(((n, l, m) for (n, l, m) in sorted(first_seen, key=first_seen.get)
                  if n == "motor_serial" and "Serial" in m), None)
        problems.append((1, f"Arduino serial link failed: {serial_err} serial "
                            f"errors from motor_serial, first at "
                            f"{hms(first_seen[k]) if k else '?'}. Odometry "
                            "(encoders) and motor commands stop when this happens."))
    usb = [l for l in kernel if "USB disconnect" in l]
    if usb:
        problems.append((1, f"{len(usb)} USB disconnect(s) in kernel log, e.g. "
                            f"`{usb[0][:120]}` (power dip / loose cable / "
                            "motor noise?)"))
    if any("2fef" in l for l in kernel):
        problems.append((1, "The Arduino re-enumerated as `03eb:2fef` "
                            "(ATmega16U2 DFU bootloader) instead of a serial "
                            "port: it needs a power-cycle / replug to come back."))
    acm = col(sysrows, "arduino_ttyACM0")
    if acm and acm[0] == 0:
        problems.append((1, "/dev/ttyACM0 (Arduino) was missing when the run "
                            "started: motors and odometry cannot work."))
    elif acm and min(acm) == 0:
        gone = next(fnum(r["t"]) for r in sysrows if r["arduino_ttyACM0"] == "0")
        problems.append((1, f"/dev/ttyACM0 (Arduino) disappeared at {hms(gone)}."))
    lid = col(sysrows, "lidar")
    if lid and min(lid) == 0:
        problems.append((1, "The lidar USB serial device disappeared during the run."))

    # ---- Pi health
    temps = col(sysrows, "temp_c")
    freqs = col(sysrows, "cpu_mhz")
    loads = col(sysrows, "load1")
    if temps and max(temps) >= 80:
        hot = sum(1 for t in temps if t >= 80) * 100 // len(temps)
        problems.append((2, f"Pi CPU temperature reached {max(temps):.0f} °C "
                            f"(≥80 °C for {hot}% of the run): it throttles "
                            "(slows down) above ~80 °C. Add a fan/heatsink."))
    # Low clock while idle is normal power saving; low clock while busy
    # means throttling.
    busy_slow = [fnum(r["cpu_mhz"]) for r in sysrows
                 if fnum(r.get("cpu_mhz")) and fnum(r.get("load1"))
                 and fnum(r["load1"]) > 2.0
                 and fnum(r["cpu_mhz"]) < 0.8 * max(freqs)]
    if busy_slow:
        problems.append((2, f"CPU clock dropped to {min(busy_slow):.0f} MHz "
                            f"(max {max(freqs):.0f} MHz) while busy, in "
                            f"{len(busy_slow)} samples: throttling."))
    for k in ("throttled_start", "throttled_end"):
        v = meta.get(k, "?")
        try:
            bits = int(v, 16)
        except ValueError:
            continue
        flags = []
        if bits & 0x1 or bits & 0x10000:
            flags.append("UNDER-VOLTAGE")
        if bits & 0x4 or bits & 0x40000:
            flags.append("throttled")
        if bits & 0x8 or bits & 0x80000:
            flags.append("soft temperature limit")
        if flags:
            problems.append((2, f"Pi firmware flags at {k.split('_')[1]} "
                                f"({v}): {', '.join(flags)}."))
    if loads and pct(loads, 0.5) > 3.5:
        problems.append((2, f"Pi CPU overloaded: median load {pct(loads, .5):.1f} "
                            "on 4 cores."))

    # ---- Nav2 / localization symptoms from logs
    def count(pattern, node=None):
        return sum(n for (nd, _, m), n in hist.items()
                   if re.search(pattern, m) and (node is None or nd == node))

    missed = count(r"missed its desired rate|tick rate .* was exceeded")
    if missed:
        problems.append((2, f"{missed} Nav2 'loop missed its desired rate' / "
                            "BT tick warnings: the Pi is not keeping up."))
    extrap = count(r"extrapolation")
    if extrap:
        problems.append((3, f"{extrap} TF 'extrapolation' warnings (late TF / "
                            "clock offset between machines)."))
    qos = count(r"incompatible QoS")
    if qos:
        topics = {re.search(r"topic '([^']+)'", m).group(1)
                  for (_, _, m) in hist if "incompatible QoS" in m
                  and re.search(r"topic '([^']+)'", m)}
        problems.append((3, f"QoS mismatch on {', '.join(sorted(topics))}: RViz "
                            "shows nothing for it (set Reliability to match)."))
    collide = count(r"collision ahead|Collision Ahead")
    if collide:
        problems.append((3, f"{collide} 'collision ahead' warnings: costmap sees "
                            "obstacles in the path (wrong pose, footprint, or "
                            "lidar seeing the robot itself)."))
    patience = count(r"patience exceeded|failed to create plan|Aborting handle")
    if patience:
        problems.append((3, f"{patience} aborted controller/planner/behavior "
                            "actions."))

    # ---- monitor-based localization diagnostics
    run_rows = [r for r in mon if fnum(r["t"]) and fnum(r["t"]) < stop_ts]
    for key, label, expect in (("scan_hz", "/scan", 5), ("odom_hz", "/odom", 10),
                               ("enc_hz", "/encoder_counts", 10)):
        vals = col(run_rows, key)
        if not vals:
            continue
        if max(vals) == 0:
            problems.append((1, f"No {label} messages seen by the monitor at all."))
            continue
        active = vals[next(i for i, v in enumerate(vals) if v > 0):]
        zero = sum(1 for v in active if v == 0)
        if zero:
            problems.append((2, f"{label} silent for {zero} s of the run "
                                f"(median {statistics.median(active):.0f} Hz)."))
        elif statistics.median(active) < expect:
            problems.append((3, f"{label} only {statistics.median(active):.0f} Hz "
                                "(median)."))
    ages = col(run_rows, "scan_age_ms")
    if ages and pct(ages, 0.9) > 200:
        problems.append((2, f"Scans arrive late: 90% delay {pct(ages, .9):.0f} ms "
                            "(AMCL matches old scans against the current pose)."))
    stds = col(run_rows, "amcl_std_xy")
    if stds:
        bad = sum(1 for s in stds if s > 0.5) * 100 // len(stds)
        if bad > 10:
            problems.append((2, f"AMCL was uncertain (std > 0.5 m) for {bad}% "
                                "of the time it published."))
    jumps = [e for e in events if e[1].startswith("AMCL_JUMP")]
    if jumps:
        problems.append((2, f"{len(jumps)} AMCL pose jump(s) (> {0.5} m or "
                            "> 29°) without a new initial pose."))
    stuck = [e for e in events if e[1].startswith("NOT_MOVING")]
    if stuck:
        problems.append((1, f"{len(stuck)} time(s) Nav2 commanded motion but "
                            "odometry did not change (motors/encoders/Arduino)."))
    inits = [e for e in events if e[1].startswith("INITIAL_POSE")]
    if len(inits) >= 3:
        problems.append((3, f"Initial pose was set {len(inits)} times: "
                            "localization was being re-done by hand."))

    # ---- laptop RViz
    laptops = laptop_reports(start, end)
    for path, lm in laptops:
        rc = lm.get("exit_code", "?")
        meaning = lm.get("exit_meaning", "")
        if rc not in ("0", "130", "?"):
            problems.append((1, f"Laptop RViz ({os.path.basename(path)}) ended "
                                f"with exit code {rc}: {meaning}"))
        samples = read_csv(os.path.join(path, "samples.csv"))
        cpu = col(samples, "cpu_pct")
        if rc == "137" and len(cpu) >= 3 and max(cpu[-3:]) <= 1:
            frozen_at = next((fnum(samples[i]["t"]) for i in range(len(samples))
                              if all((fnum(r.get("cpu_pct")) or 0) <= 1
                                     for r in samples[i:])
                              and fnum(samples[i].get("cpu_pct")) is not None), None)
            problems.append((1, "Laptop RViz FROZE (blocked at ~0% CPU"
                                + (f" from {hms(frozen_at)}" if frozen_at else "")
                                + ") and was force-quit."))
        rss = col(samples, "rss_mb")
        if rss and max(rss) > 1.5 * max(rss[0], 1) and max(rss) > 800:
            problems.append((2, f"RViz memory grew {rss[0]:.0f} → {max(rss):.0f} MB."))
        pings = [r.get("ping_ms", "") for r in samples]
        lost = sum(1 for p in pings if p in ("", "lost"))
        if pings and lost:
            problems.append((2, f"Laptop lost ping to the Pi in {lost}/{len(pings)} "
                                "samples (Wi-Fi / hotspot drops)."))
        lat = [x for x in (fnum(p) for p in pings) if x is not None]
        if lat and pct(lat, 0.9) > 100:
            problems.append((3, f"Wi-Fi latency laptop→Pi high: 90% {pct(lat, .9):.0f} ms."))
    if not laptops:
        problems.append((4, "No laptop RViz report found for this run (it is "
                            "uploaded when `rviz` exits on the laptop)."))

    # ---------------------------------------------------------------- write
    out = []
    w = out.append
    w(f"# Run report {os.path.basename(os.path.realpath(run))}\n")
    w(f"- Start: {meta.get('start', '?')}  End: {meta.get('end', '?')}  "
      f"Duration: {int((end - start) // 60)} min {int((end - start) % 60)} s")
    w(f"- Command: `{meta.get('command', '?')}`")
    w(f"- Ended by: {meta.get('ended_by', '?')} (launch exit code "
      f"{meta.get('launch_exit_code', '?')})")
    w(f"- Network: {meta.get('rmw', '?')}, DDS peers: {meta.get('dds_peers', '?')}"
      f", SSH from {meta.get('ssh_client', '?')}")
    w(f"- Code: git {meta.get('git', '?')}\n")

    w("## Detected problems (most serious first)\n")
    sev_name = {1: "🔴", 2: "🟠", 3: "🟡", 4: "⚪"}
    for sev, text in sorted(problems, key=lambda p: p[0]):
        w(f"- {sev_name[sev]} {text}")
    if not problems:
        w("- None detected.")
    w("")

    # timeline
    tl = [(t, f"LAUNCH {who}: {m[:140]}") for t, _, who, m in launch_events
          if "process started" not in m]
    tl += [(t, f"EVENT {e}") for t, e in events]
    tl += timeline
    for l in kernel:
        m = re.match(r"(\S+)", l)
        try:
            ts = time.mktime(time.strptime(m.group(1)[:19], "%Y-%m-%dT%H:%M:%S"))
            tl.append((ts, "KERNEL " + l[len(m.group(1)):].strip()[:150]))
        except (ValueError, AttributeError):
            pass
    for path, lm in laptops:
        for t, txt in ((fnum(lm.get("start_epoch")), "LAPTOP rviz started"),
                       (fnum(lm.get("end_epoch")),
                        f"LAPTOP rviz exited code {lm.get('exit_code')} "
                        f"{lm.get('exit_meaning', '')}")):
            if t:
                tl.append((t, txt))
    tl.sort()
    w("## Timeline (first occurrence of each warning type, events)\n")
    w("```")
    shown = tl if len(tl) <= 150 else tl[:100] + [(0, "...")] + tl[-50:]
    for t, txt in shown:
        w(f"{hms(t) if t else '        '}  {txt}")
    w("```\n")

    # warning histogram
    w("## Warnings / errors by node (count, normalized message)\n")
    by_node = defaultdict(list)
    for (node, level, msg), n in hist.items():
        by_node[node].append((n, level, msg))
    for node in sorted(by_node, key=lambda k: -sum(x[0] for x in by_node[k])):
        w(f"**{node}**\n")
        w("```")
        for n, level, msg in sorted(by_node[node], reverse=True)[:12]:
            w(f"{n:7d} {level:5s} {msg}")
        w("```")
    w("")

    # topic stats
    if run_rows:
        w("## Topic health while running (from monitor.csv, 1 s windows)\n")
        w("| signal | median | min | 90th pct |")
        w("|---|---|---|---|")
        for key in ("scan_hz", "odom_hz", "enc_hz", "cmd_hz", "amcl_hz",
                    "scan_age_ms", "scan_valid_pct", "amcl_std_xy",
                    "amcl_std_yaw", "tf_map_base_age_ms", "tf_odom_base_age_ms"):
            v = col(run_rows, key)
            if v:
                w(f"| {key} | {statistics.median(v):.2f} | {min(v):.2f} | "
                  f"{pct(v, .9):.2f} |")
        w("")
    if sysrows:
        w("## Pi system\n")
        w(f"- Temperature: max {max(temps):.1f} °C, median "
          f"{statistics.median(temps):.1f} °C" if temps else "- Temperature: ?")
        if freqs:
            w(f"- CPU clock: min {min(freqs):.0f} MHz, max {max(freqs):.0f} MHz")
        if loads:
            w(f"- Load (1 min): median {statistics.median(loads):.2f}, "
              f"max {max(loads):.2f}")
        tops = Counter()
        for r in sysrows:
            for item in r.get("top_cpu", "").split():
                name, _, cpu = item.rpartition(":")
                if fnum(cpu) is not None:
                    tops[name] += fnum(cpu)
        if tops:
            w("- Avg CPU% of busiest processes: " + ", ".join(
                f"{n} {v / len(sysrows):.0f}%" for n, v in tops.most_common(6)))
        w(f"- Firmware throttle flags: start {meta.get('throttled_start')}, "
          f"end {meta.get('throttled_end')}")
        w("")

    for path, lm in laptops:
        w(f"## Laptop RViz: {os.path.basename(path)}\n")
        for k in ("start", "end", "exit_code", "exit_meaning", "laptop_ip",
                  "pi", "rviz_display_server", "gpu"):
            if k in lm:
                w(f"- {k}: {lm[k]}")
        samples = read_csv(os.path.join(path, "samples.csv"))
        if samples:
            rss = col(samples, "rss_mb")
            cpu = col(samples, "cpu_pct")
            w(f"- RViz RSS: start {rss[0]:.0f} MB, max {max(rss):.0f} MB; CPU "
              f"median {statistics.median(cpu):.0f}%, max {max(cpu):.0f}%"
              if rss and cpu else "")
            w("- Last samples before exit:\n```")
            for r in samples[-8:]:
                w(", ".join(f"{k}={v}" for k, v in r.items()))
            w("```")
        for auto in sorted(glob.glob(os.path.join(path, "auto_threads_*.txt"))):
            w(f"- RViz blocked (~0% CPU), automatic thread snapshot "
              f"`{os.path.basename(auto)}`:\n```")
            w("\n".join(read_lines(auto)[:60]))
            w("```")
        hang = os.path.join(path, "hang_threads.txt")
        if os.path.exists(hang):
            w("- Thread dump taken before force-kill (`rviz-hang`):\n```")
            w("\n".join(read_lines(hang)[:80]))
            w("```")
        lc = read_lines(os.path.join(path, "console.log"))
        bad = [l for l in lc if re.search(r"ERROR|WARN|error|fault|abort|what\(\)", l)]
        w("- RViz warnings/errors (deduplicated):\n```")
        for msg, n in Counter(normalize(re.sub(r"^\[\w+\] \[[0-9.]+\] ", "", l))
                              for l in bad).most_common(20):
            w(f"{n:6d} {msg}")
        w("```")
        w("- RViz console tail:\n```")
        w("\n".join(lc[-25:]))
        w("```\n")

    w("## Console tail (last 40 lines)\n")
    w("```")
    w("\n".join(l[:250] for l in console[-40:]))
    w("```")

    report = "\n".join(out) + "\n"
    with open(os.path.join(run, "REPORT.md"), "w") as f:
        f.write(report)
    return problems


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    run = os.path.realpath(args[0] if args else os.path.join(REPORTS, "latest"))
    problems = build(run)
    if "--print-summary" in sys.argv:
        print(f"[report] {len(problems)} finding(s):")
        for sev, text in sorted(problems, key=lambda p: p[0])[:12]:
            print(f"  {'!' * (4 - min(sev, 3))} {text.splitlines()[0][:160]}")


if __name__ == "__main__":
    main()
