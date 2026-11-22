#!/bin/bash
# Start the buggy ROS 2 desktop in Docker (builds the image on first run).
#
# Opens an SSH tunnel Mac:7447 -> Pi:7447 first, so the container reaches
# the Pi's Zenoh bridge via host.docker.internal even when macOS blocks
# local IPv4 traffic.
set -e
cd "$(dirname "$0")"

set -a
. ./.env
set +a

if [ "${PI_ADDR}" = "host.docker.internal" ]; then
    if nc -z 127.0.0.1 7447 2>/dev/null; then
        echo "SSH tunnel to the Pi is already open."
    else
        echo "Opening SSH tunnel to ${PI_SSH} (enter the Pi's password)..."
        ssh -f -N \
            -o ExitOnForwardFailure=yes \
            -o ServerAliveInterval=15 \
            -o ServerAliveCountMax=3 \
            -L 127.0.0.1:7447:127.0.0.1:7447 \
            "${PI_SSH}"
        echo "Tunnel open."
    fi
fi

docker compose up -d --build

echo
echo "Desktop is starting. In Windows App:"
echo "  Add PC -> PC name: localhost:3390"
echo "  User: ros   Password: ros"
echo "RViz opens automatically after login."
echo
echo "Check the Pi connection:  docker compose logs --tail 20"
echo "Stop everything:          ./stop.sh"
