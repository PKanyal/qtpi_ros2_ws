#!/bin/bash
# Container start: RDP desktop server, then the Zenoh bridge to the Pi.
set -e

if [ -z "${PI_ADDR:-}" ]; then
    echo "ERROR: PI_ADDR is not set (edit .env next to docker-compose.yml)" >&2
    exit 1
fi

# Fresh start for xrdp after a container restart
rm -f /var/run/xrdp/*.pid /var/run/xrdp.pid
mkdir -p /var/run/dbus && rm -f /var/run/dbus/pid
dbus-daemon --system --fork
/usr/sbin/xrdp-sesman
/usr/sbin/xrdp

echo "[buggy] Desktop ready: Windows App -> localhost:3390 (user ros / password ros)"

source /etc/profile.d/ros.sh

if ! zenoh-bridge-ros2dds --version >/dev/null 2>&1; then
    echo "[buggy] ERROR: zenoh-bridge-ros2dds cannot run in this container." >&2
    echo "  container CPU: $(uname -m)" >&2
    echo "  binary header: $(head -c 20 /usr/local/bin/zenoh-bridge-ros2dds | od -An -tx1 | head -1)" >&2
    echo "  Rebuild the image: docker compose build --no-cache" >&2
fi

# Keep the bridge running; reconnect if the Pi restarts or is not up yet.
while true; do
    if timeout 3 bash -c "</dev/tcp/${PI_ADDR}/7447" 2>/dev/null; then
        echo "[buggy] connecting Zenoh bridge to tcp/${PI_ADDR}:7447"
        zenoh-bridge-ros2dds client -e "tcp/${PI_ADDR}:7447" \
            --no-multicast-scouting || true
        echo "[buggy] bridge stopped, retrying in 3 s"
    else
        echo "[buggy] cannot reach ${PI_ADDR}:7447 (navigation running on the Pi?), retrying in 5 s"
        sleep 2
    fi
    sleep 3
done
