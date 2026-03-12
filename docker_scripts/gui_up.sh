#!/usr/bin/env bash
set -e

# Allow local docker containers to use your X server (X11)
xhost +local:docker >/dev/null

export HOSTNAME=$(hostname)

docker compose up -d --build
docker compose exec ros2 bash
