#!/bin/bash
# Stop the buggy ROS 2 desktop and close the SSH tunnel to the Pi.
cd "$(dirname "$0")"
docker compose down
pkill -f "127.0.0.1:7447:127.0.0.1:7447" && echo "SSH tunnel closed." || true
