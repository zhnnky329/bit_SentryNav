#!/usr/bin/env zsh
# Copyright 2026 bw_nav contributors
# SPDX-License-Identifier: Apache-2.0
#
# Requires zsh 5.x (uses zsh arrays and the %x prompt expansion).
#
# Live pipeline:
#   Livox MID360 driver -> FAST-LIO -> /cloud_registered_body
#   -> optional ground/obstacle filter (for the Nav2 costmaps)
#   -> relocalizer (map -> camera_init, indirect TF mode)
#
# The filter is downstream of FAST-LIO: it consumes /cloud_registered_body, so
# it is started only after FAST-LIO is actually publishing that topic.  It is
# not a prerequisite for the relocalizer itself.
#
# Usage:
#   ./tools/start_pipeline.zsh
#   ./tools/start_pipeline.zsh --plan /path/map.yaml --surface /path/map.pcd
#   ./tools/start_pipeline.zsh --no-filter --no-base-tf
#   ./tools/start_pipeline.zsh --timeout 90 --log-dir /tmp/live_pipeline
#
# Environment overrides:
#   NAV_WS, RELOCALIZATION_WS, ROS_SETUP, PLAN_FILE, SURFACE_FILE,
#   SWEEP_TOPIC, ODOM_FRAME, WAIT_TIMEOUT, START_FILTER, START_BASE_TF,
#   START_RVIZ, AUTOMATIC, LOG_DIR

set -e
set -u
set -o pipefail

SCRIPT_DIR=${${(%):-%x}:A:h}
RELOCALIZATION_WS="${RELOCALIZATION_WS:-$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)}"
NAV_WS="${NAV_WS:-$RELOCALIZATION_WS}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.zsh}"

PLAN_FILE="${PLAN_FILE:-$RELOCALIZATION_WS/maps/nav2/mid360_pgo_csf.yaml}"
SURFACE_FILE="${SURFACE_FILE:-$RELOCALIZATION_WS/maps/pgo_session/map_consistent.pcd}"
SWEEP_TOPIC="${SWEEP_TOPIC:-/cloud_registered_body}"
ODOM_FRAME="${ODOM_FRAME:-camera_init}"
WAIT_TIMEOUT="${WAIT_TIMEOUT:-60}"
START_FILTER="${START_FILTER:-1}"
START_BASE_TF="${START_BASE_TF:-1}"
START_RVIZ="${START_RVIZ:-1}"
AUTOMATIC="${AUTOMATIC:-true}"
LOG_DIR="${LOG_DIR:-}"

CHILD_PIDS=()
UPSTREAM_PIDS=()
typeset -A PID_LABEL
PID_LABEL=()
LAST_PID=""
STOPPING=0

show_help() {
  cat <<'EOF'
Live pipeline:
  Livox MID360 driver -> FAST-LIO -> /cloud_registered_body
  -> optional ground/obstacle filter (for the Nav2 costmaps)
  -> relocalizer (map -> camera_init, indirect TF mode)

The filter is downstream of FAST-LIO: it consumes /cloud_registered_body, so
it starts only after FAST-LIO is publishing that topic. It is not a
prerequisite for the relocalizer itself.

Usage:
  ./tools/start_pipeline.zsh
  ./tools/start_pipeline.zsh --plan /path/map.yaml --surface /path/map.pcd
  ./tools/start_pipeline.zsh --no-filter --no-base-tf
  ./tools/start_pipeline.zsh --timeout 90 --log-dir /tmp/live_pipeline

Options:
  --plan PATH          blocked-cell raster YAML
  --surface PATH       3-D surface map PCD
  --sweep-topic TOPIC  FAST-LIO body-frame PointCloud2 (default /cloud_registered_body)
  --odom-frame FRAME   TF bridge frame published by FAST-LIO (default camera_init)
  --timeout SEC        wait for the first cloud message (default 60)
  --log-dir DIR        log directory (default <workspace>/.pipeline/<timestamp>)
  --no-filter          do not start the Nav2 ground/obstacle filter
  --no-base-tf         do not publish body -> base_footprint
  --no-rviz            do not start RViz (headless / no X display)
  --rviz               start RViz (default)
  --manual             wait for the ~/relocalize service instead of auto-searching
  --automatic          auto-search on startup (default)
  -h, --help           show this help

Environment overrides:
  NAV_WS, RELOCALIZATION_WS, ROS_SETUP, PLAN_FILE, SURFACE_FILE,
  SWEEP_TOPIC, ODOM_FRAME, WAIT_TIMEOUT, START_FILTER, START_BASE_TF,
  START_RVIZ, AUTOMATIC, LOG_DIR
EOF
}

fatal() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

collect_descendants() {
  local parent="$1" child
  for child in ${(f)"$(pgrep -P "$parent" 2>/dev/null || true)"}; do
    [[ -n "$child" ]] || continue
    collect_descendants "$child"
    printf '%s\n' "$child"
  done
}

stop_pipeline() {
  local exit_code="${1:-0}"
  if ((STOPPING)); then
    return
  fi
  STOPPING=1
  trap - EXIT INT TERM HUP
  if ((${#CHILD_PIDS[@]})); then
    printf '\n[pipeline] stopping pipeline processes...\n'
    local -a tree
    tree=()
    local pid child
    # Snapshot the whole process tree before signalling.  Once ros2 launch
    # exits, its node is reparented and can no longer be found via pgrep -P.
    for pid in "${CHILD_PIDS[@]}"; do
      tree+=("$pid")
      while read -r child; do
        tree+=("$child")
      done < <(collect_descendants "$pid")
    done
    kill -INT "${tree[@]}" 2>/dev/null || true
    sleep 2
    kill -TERM "${tree[@]}" 2>/dev/null || true
    sleep 1
    kill -KILL "${tree[@]}" 2>/dev/null || true
    wait 2>/dev/null || true
  fi
  printf '[pipeline] logs: %s\n' "$LOG_DIR"
  exit "$exit_code"
}

launch_group() {
  local label="$1"
  shift
  local log_file="$LOG_DIR/${label}.log"
  setsid --wait "$@" >"$log_file" 2>&1 &
  LAST_PID=$!
  CHILD_PIDS+=("$LAST_PID")
  PID_LABEL[$LAST_PID]="$label"
  sleep 0.4
  if ! kill -0 "$LAST_PID" 2>/dev/null; then
    printf 'ERROR: %s exited during startup\n' "$label" >&2
    tail -n 50 "$log_file" >&2 || true
    exit 1
  fi
  printf '  %-16s pid=%-7s log=%s\n' "$label" "$LAST_PID" "$log_file"
}

ensure_upstream() {
  local pid
  for pid in "${UPSTREAM_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      local label="${PID_LABEL[$pid]:-upstream}"
      printf '\nERROR: %s exited while waiting\n' "$label" >&2
      tail -n 60 "$LOG_DIR/${label}.log" >&2 || true
      exit 1
    fi
  done
}

await_topic() {
  local topic="$1" timeout="$2"
  local started=$SECONDS
  while ((SECONDS - started < timeout)); do
    ensure_upstream
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

await_cloud() {
  local started=$SECONDS
  local info
  while ((SECONDS - started < WAIT_TIMEOUT)); do
    ensure_upstream
    info="$(ros2 topic info "$SWEEP_TOPIC" --verbose 2>/dev/null || true)"
    if grep -qE 'Publisher count: [1-9][0-9]*' <<<"$info"; then
      if ! grep -q 'Type: sensor_msgs/msg/PointCloud2' <<<"$info"; then
        printf '\nERROR: %s is not sensor_msgs/msg/PointCloud2:\n' "$SWEEP_TOPIC" >&2
        grep -E 'Type:|Publisher count:' <<<"$info" >&2 || true
        exit 1
      fi
      # A publisher exists, but wait for one real message as well.  A
      # best-effort subscription also matches FAST-LIO's default publisher.
      if timeout 5 ros2 topic echo "$SWEEP_TOPIC" --once \
          --qos-reliability best_effort >/dev/null 2>&1; then
        return 0
      fi
    fi
    printf '.' >&2
    sleep 1
  done
  return 1
}

while (($#)); do
  case "$1" in
    --plan)
      [[ $# -ge 2 ]] || fatal "--plan needs a path"
      PLAN_FILE="$2"
      shift 2
      ;;
    --surface)
      [[ $# -ge 2 ]] || fatal "--surface needs a path"
      SURFACE_FILE="$2"
      shift 2
      ;;
    --sweep-topic)
      [[ $# -ge 2 ]] || fatal "--sweep-topic needs a topic"
      SWEEP_TOPIC="$2"
      shift 2
      ;;
    --odom-frame)
      [[ $# -ge 2 ]] || fatal "--odom-frame needs a frame"
      ODOM_FRAME="$2"
      shift 2
      ;;
    --timeout)
      [[ $# -ge 2 ]] || fatal "--timeout needs seconds"
      WAIT_TIMEOUT="$2"
      shift 2
      ;;
    --log-dir)
      [[ $# -ge 2 ]] || fatal "--log-dir needs a path"
      LOG_DIR="$2"
      shift 2
      ;;
    --no-filter)
      START_FILTER=0
      shift
      ;;
    --no-base-tf)
      START_BASE_TF=0
      shift
      ;;
    --no-rviz)
      START_RVIZ=0
      shift
      ;;
    --rviz)
      START_RVIZ=1
      shift
      ;;
    --manual)
      AUTOMATIC=false
      shift
      ;;
    --automatic)
      AUTOMATIC=true
      shift
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      fatal "unknown argument: $1"
      ;;
  esac
done

[[ "$WAIT_TIMEOUT" =~ ^[0-9]+$ ]] || fatal "--timeout must be an integer"
((WAIT_TIMEOUT > 0)) || fatal "--timeout must be positive"
[[ -f "$ROS_SETUP" ]] || fatal "ROS setup not found: $ROS_SETUP"
[[ -f "$NAV_WS/install/setup.zsh" ]] || \
  fatal "nav workspace is not built: $NAV_WS/install/setup.zsh"
[[ -f "$RELOCALIZATION_WS/install/setup.zsh" ]] || \
  fatal "relocalizer workspace is not built: $RELOCALIZATION_WS/install/setup.zsh"
[[ -f "$PLAN_FILE" ]] || fatal "plan descriptor not found: $PLAN_FILE"
[[ -f "$SURFACE_FILE" ]] || fatal "surface map not found: $SURFACE_FILE"
command -v setsid >/dev/null 2>&1 || fatal "setsid is required"
command -v timeout >/dev/null 2>&1 || fatal "timeout is required"

if [[ -z "$LOG_DIR" ]]; then
  LOG_DIR="$RELOCALIZATION_WS/.pipeline/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$LOG_DIR"
LOG_DIR="$(cd "$LOG_DIR" && pwd)"

# Some sandboxed sessions mount /home/pgd/.ros read-only. Keep ROS logs and the
# ROS home beside the pipeline logs so ros2 launch always has a writable dir.
export ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/roslog}"
export ROS_HOME="${ROS_HOME:-$LOG_DIR/roshome}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"

trap 'stop_pipeline $?' EXIT
trap 'stop_pipeline 130' INT
trap 'stop_pipeline 143' TERM
trap 'stop_pipeline 129' HUP

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ "$NAV_WS" != "$RELOCALIZATION_WS" ]]; then
  source "$NAV_WS/install/setup.zsh"
fi
source "$RELOCALIZATION_WS/install/setup.zsh"
set -u

for pkg in livox_ros_driver2 fast_lio relocalizer; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fatal "ROS package not found: $pkg"
done
if [[ "$START_FILTER" == "1" ]]; then
  ros2 pkg prefix bw_fastlio_nav2 >/dev/null 2>&1 || \
    fatal "ROS package not found: bw_fastlio_nav2 (use --no-filter to skip it)"
fi

FASTLIO_PREFIX="$(ros2 pkg prefix fast_lio)"
FASTLIO_PARAMS="$FASTLIO_PREFIX/share/fast_lio/config/mid360.yaml"
[[ -f "$FASTLIO_PARAMS" ]] || fatal "FAST-LIO config not found: $FASTLIO_PARAMS"

# Parameters that differ from the filter node's defaults; they mirror the
# production Nav2 launch for the MID360 on this chassis.
FILTER_PARAMS=(
  -p "input_topic:=$SWEEP_TOPIC"
  -p "grid_topic:=/obstacle_grid_live"
  -p "ground_topic:=/ground_points_live"
  -p "obstacle_topic:=/obstacle_points_live"
  -p "voxel_leaf_size:=0.05"
  -p "sensor_height:=0.18"
  -p "num_iter:=3"
  -p "num_lpr:=20"
  -p "num_min_points:=10"
  -p "seed_threshold:=0.12"
  -p "distance_threshold:=0.02"
  -p "vertical_seed_threshold:=0.25"
  -p "vertical_distance_threshold:=0.02"
  -p "min_range:=0.05"
  -p "max_range:=15.0"
  -p "uprightness_threshold:=0.866"
  -p "enable_reflected_noise_removal:=false"
  -p "enable_vertical_plane_fitting:=true"
  -p "enable_temporal_ground_revert:=true"
  -p "obstacle_min_height:=0.03"
  -p "obstacle_max_height:=0.30"
  -p "outlier_radius:=0.20"
  -p "outlier_min_neighbors:=2"
  -p "resolution:=0.05"
  -p "min_obstacle_neighbors:=3"
  -p "min_update_period_sec:=0.10"
  -p "self_filter_enabled:=true"
  -p "self_filter_min_x:=-0.45"
  -p "self_filter_max_x:=0.0"
  -p "self_filter_min_y:=-0.25"
  -p "self_filter_max_y:=0.25"
  -p "fov_filter_enabled:=true"
  -p "fov_obstacle_topic:=/obstacle_points_fov"
  -p "fov_min_azimuth_deg:=-135.0"
  -p "fov_max_azimuth_deg:=135.0"
  -p "fov_min_elevation_deg:=-90.0"
  -p "fov_max_elevation_deg:=20.0"
  -p "fov_min_range:=0.05"
  -p "fov_max_range:=15.0"
  -p "fov_mount_roll_deg:=0.0"
  -p "fov_mount_pitch_deg:=0.0"
  -p "fov_mount_yaw_deg:=0.0"
)

printf '== live relocalization pipeline ==\n'
printf '  nav workspace:  %s\n' "$NAV_WS"
printf '  relocalizer:    %s\n' "$RELOCALIZATION_WS"
printf '  plan:           %s\n' "$PLAN_FILE"
printf '  surface:        %s\n' "$SURFACE_FILE"
printf '  cloud topic:    %s\n' "$SWEEP_TOPIC"
printf '  odom frame:     %s\n' "$ODOM_FRAME"
printf '  logs:           %s\n\n' "$LOG_DIR"

printf '[1/5] Livox MID360 driver\n'
launch_group lidar_driver ros2 launch livox_ros_driver2 msg_MID360s_launch.py
UPSTREAM_PIDS+=("$LAST_PID")

printf '[2/5] waiting for /livox/lidar and /livox/imu\n'
if ! await_topic /livox/lidar 20; then
  fatal "driver did not advertise /livox/lidar (see $LOG_DIR/lidar_driver.log)"
fi
if ! await_topic /livox/imu 20; then
  fatal "driver did not advertise /livox/imu (see $LOG_DIR/lidar_driver.log)"
fi

printf '\n[3/5] FAST-LIO\n'
launch_group fast_lio ros2 run fast_lio fastlio_mapping \
  --ros-args --params-file "$FASTLIO_PARAMS" \
  -p pcd_save.pcd_save_en:=false
UPSTREAM_PIDS+=("$LAST_PID")

printf '\n[4/5] waiting for %s (up to %ss)\n' "$SWEEP_TOPIC" "$WAIT_TIMEOUT"
if ! await_cloud; then
  printf '\n'
  tail -n 80 "$LOG_DIR/fast_lio.log" >&2 || true
  fatal "no %s message from FAST-LIO; check the LiDAR link and /livox/lidar" "$SWEEP_TOPIC"
fi
printf '\n  %s is publishing PointCloud2\n' "$SWEEP_TOPIC"

if [[ "$START_FILTER" == "1" ]]; then
  printf '[5/5] starting ground/obstacle filter (downstream of FAST-LIO)\n'
  launch_group obstacle_filter ros2 run bw_fastlio_nav2 laser_map_ground_grid \
    --ros-args "${FILTER_PARAMS[@]}"
fi

if [[ "$START_BASE_TF" == "1" ]]; then
  printf '       starting body -> base_footprint static TF\n'
  launch_group base_tf ros2 run tf2_ros static_transform_publisher \
    --x -0.14 --y 0.0 --z -0.18 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id body --child-frame-id base_footprint
fi

RVIZ_ENABLED=false
if [[ "$START_RVIZ" == "1" ]]; then
  RVIZ_ENABLED=true
  if [[ -z "${DISPLAY:-}" ]]; then
    printf 'WARNING: DISPLAY is empty; RViz needs an X11 display.\n' >&2
    printf '         Use --no-rviz, connect with ssh -X, or run from the robot desktop.\n' >&2
  fi
fi

printf '\n[relocalizer] starting (Ctrl-C stops the whole pipeline)\n'
printf '             publishing map -> %s, FAST-LIO owns %s -> body\n' \
  "$ODOM_FRAME" "$ODOM_FRAME"
printf '             status: ros2 topic echo /relocalizer/status\n'
printf '             live log: %s\n\n' "$LOG_DIR/relocalizer.log"

setsid --wait ros2 launch relocalizer relocalizer.launch.py \
  plan_descriptor:="$PLAN_FILE" \
  surface_map:="$SURFACE_FILE" \
  sweep_topic:="$SWEEP_TOPIC" \
  body_frame:=body \
  world_frame:=map \
  odom_frame:="$ODOM_FRAME" \
  automatic:="$AUTOMATIC" \
  rviz:="$RVIZ_ENABLED" > >(tee "$LOG_DIR/relocalizer.log") 2>&1 &
RELOCALIZER_PID=$!
CHILD_PIDS+=("$RELOCALIZER_PID")
PID_LABEL[$RELOCALIZER_PID]="relocalizer"

set +e
wait "$RELOCALIZER_PID"
exit_code=$?
set -e
exit "$exit_code"
