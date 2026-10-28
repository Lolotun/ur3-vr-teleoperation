#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

case "${1:-help}" in
  servo|sim)
    TELEOP_MODE=servo_sim docker compose up -d --no-build
    echo "Virtual UR3 mode: MoveIt Servo; no robot or gripper bridges are launched."
    ;;
  up|wifi-speedl|rtde)
    # Compose reads .env; do not source it as executable shell code.
    docker compose config --format json | python3 -c '
import json, sys
env = json.load(sys.stdin)["services"]["teleop"]["environment"]
if not env.get("ROBOT_IP"):
    sys.exit("Set ROBOT_IP in .env or the environment before starting.")
'
    if [[ "$1" == "rtde" ]]; then
      TELEOP_MODE=rtde_real docker compose up -d --no-build
      echo "RTDE mode: preview unless RTDE_ENABLE_MOTION=true in .env."
    else
      TELEOP_MODE=wifi_speedl docker compose up -d --no-build
    fi
    ;;
  build) docker compose build ;;
  down) docker compose down ;;
  logs) docker compose logs -f teleop ;;
  ps) docker compose ps ;;
  shell) docker compose exec teleop bash ;;
  topics)
    docker compose exec teleop bash -lc \
      'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic list'
    ;;
  help|-h|--help)
    echo "Usage: $0 {build|servo|sim|up|wifi-speedl|rtde|down|logs|ps|shell|topics}"
    ;;
  *) echo "Unknown command: $1. Run $0 help." >&2; exit 1 ;;
esac
