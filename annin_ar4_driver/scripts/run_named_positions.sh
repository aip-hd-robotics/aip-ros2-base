#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_named_positions.sh [--file PATH] [--pause SECONDS] POSE [POSE ...]

Examples:
  run_named_positions.sh home upright home
  run_named_positions.sh --pause 1.5 --file /path/to/named_poses.yaml home upright
EOF
}

pose_file="$(ros2 pkg prefix --share annin_ar4_driver)/config/named_poses.yaml"
pause_seconds="2"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --file)
      if [[ $# -lt 2 ]]; then
        usage
        exit 1
      fi
      pose_file="$2"
      shift 2
      ;;
    --pause)
      if [[ $# -lt 2 ]]; then
        usage
        exit 1
      fi
      pause_seconds="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -* )
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -eq 0 ]]; then
  usage
  exit 1
fi

for pose_name in "$@"; do
  ros2 run annin_ar4_driver go_to_named_position.py \
    --file "$pose_file" \
    --name "$pose_name"

  if [[ "$pause_seconds" != "0" ]]; then
    sleep "$pause_seconds"
  fi
done