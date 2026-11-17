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
#   ros2 run relocalizer start_pipeline.zsh --nav-setup /path/install/setup.zsh \
#     --plan /path/map.yaml --surface /path/map.pcd
#   ros2 run relocalizer start_pipeline.zsh --plan /path/map.yaml --surface /path/map.pcd
#   ros2 run relocalizer start_pipeline.zsh --no-filter --no-base-tf
#   ros2 run relocalizer start_pipeline.zsh --timeout 90 --log-dir /tmp/live_pipeline
#
# Environment overrides:
#   NAV_WS, NAV_SETUP, RELOCALIZATION_SETUP, ROS_SETUP,
#   PLAN_FILE, SURFACE_FILE, LIVOX_LAUNCH, FASTLIO_PARAMS, FILTER_CONFIG,
#   SWEEP_TOPIC, BODY_FRAME, WORLD_FRAME, ODOM_FRAME, WAIT_TIMEOUT,
#   START_FILTER, START_BASE_TF, START_RVIZ, AUTOMATIC, LOG_DIR

set -e
set -u
set -o pipefail

NAV_WS="${NAV_WS:-}"
NAV_SETUP="${NAV_SETUP:-${NAV_WS:+$NAV_WS/install/setup.zsh}}"
RELOCALIZATION_SETUP="${RELOCALIZATION_SETUP:-}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.zsh}"

PLAN_FILE="${PLAN_FILE:-}"
SURFACE_FILE="${SURFACE_FILE:-}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360s_launch.py}"
FASTLIO_PARAMS="${FASTLIO_PARAMS:-}"
FILTER_CONFIG="${FILTER_CONFIG:-}"
SWEEP_TOPIC="${SWEEP_TOPIC:-/cloud_registered_body}"
BODY_FRAME="${BODY_FRAME:-body}"
WORLD_FRAME="${WORLD_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME-camera_init}"
BASE_TRANSFORM=(-0.14 0.0 -0.18 0.0 0.0 0.0)
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
  ros2 run relocalizer start_pipeline.zsh --nav-setup /path/install/setup.zsh \
    --plan /path/map.yaml --surface /path/map.pcd
  ros2 run relocalizer start_pipeline.zsh --no-filter --no-base-tf
  ros2 run relocalizer start_pipeline.zsh --timeout 90 --log-dir /tmp/live_pipeline

Options:
  --nav-setup PATH     upstream Jazzy setup.zsh (optional if packages already sourced)
  --relocalizer-setup PATH  relocalizer setup.zsh (optional if package already sourced)
  --plan PATH          blocked-cell raster YAML
  --surface PATH       3-D surface map PCD
  --livox-launch FILE  driver launch (default msg_MID360s_launch.py; configure its LiDAR IPs)
  --fastlio-params PATH  FAST-LIO YAML (default installed fast_lio/config/mid360.yaml)
  --filter-params PATH  ground/obstacle filter YAML (default existing chassis parameters)
  --sweep-topic TOPIC  FAST-LIO body-frame PointCloud2 (default /cloud_registered_body)
  --body-frame FRAME   input cloud / upstream TF child (default body)
  --world-frame FRAME  output map frame (default map)
  --odom-frame FRAME   TF bridge frame published by FAST-LIO (default camera_init)
  --base-transform X Y Z ROLL PITCH YAW  body -> base_footprint, meters/radians
  --timeout SEC        wait for the first cloud message (default 60)
  --log-dir DIR        log directory (default <current directory>/.pipeline/<timestamp>)
  --no-filter          do not start the Nav2 ground/obstacle filter
  --no-base-tf         do not publish body -> base_footprint
  --no-rviz            do not start RViz (headless / no X display)
  --rviz               start RViz (default)
  --manual             wait for the ~/relocalize service instead of auto-searching
  --automatic          auto-search on startup (default)
  -h, --help           show this help

Environment overrides:
  NAV_WS, NAV_SETUP, RELOCALIZATION_SETUP, ROS_SETUP,
  PLAN_FILE, SURFACE_FILE, LIVOX_LAUNCH, FASTLIO_PARAMS, FILTER_CONFIG,
  SWEEP_TOPIC, BODY_FRAME, WORLD_FRAME, ODOM_FRAME, WAIT_TIMEOUT,
  START_FILTER, START_BASE_TF, START_RVIZ, AUTOMATIC, LOG_DIR

NAV_WS selects <NAV_WS>/install/setup.zsh. Use --nav-setup for other layouts.
All upstream overlays must be built against Jazzy. Configure the driver IPs,
FAST-LIO topics/frames/extrinsics and base transform for the actual robot.
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
    --nav-setup|--relocalizer-setup|--livox-launch|--fastlio-params|--filter-params|--body-frame|--world-frame)
      [[ $# -ge 2 ]] || fatal "$1 needs a value"
      case "$1" in
        --nav-setup) NAV_SETUP="$2" ;;
        --relocalizer-setup) RELOCALIZATION_SETUP="$2" ;;
        --livox-launch) LIVOX_LAUNCH="$2" ;;
        --fastlio-params) FASTLIO_PARAMS="$2" ;;
        --filter-params) FILTER_CONFIG="$2" ;;
        --body-frame) BODY_FRAME="$2" ;;
        --world-frame) WORLD_FRAME="$2" ;;
      esac
      shift 2
      ;;
    --base-transform)
      [[ $# -ge 7 ]] || fatal "--base-transform needs X Y Z ROLL PITCH YAW"
      BASE_TRANSFORM=("$2" "$3" "$4" "$5" "$6" "$7")
      shift 7
      ;;
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
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == jazzy ]] || \
  fatal "ROS_DISTRO=$ROS_DISTRO; open a clean Jazzy terminal"
[[ -n "$ODOM_FRAME" && "$ODOM_FRAME" != "$BODY_FRAME" && "$ODOM_FRAME" != "$WORLD_FRAME" ]] || \
  fatal "the FAST-LIO pipeline needs a distinct, non-empty --odom-frame"
[[ -f "$ROS_SETUP" ]] || fatal "ROS setup not found: $ROS_SETUP"
if [[ -n "$NAV_SETUP" ]]; then
  [[ -f "$NAV_SETUP" ]] || fatal "upstream Jazzy setup not found: $NAV_SETUP"
fi
if [[ -n "$RELOCALIZATION_SETUP" ]]; then
  [[ -f "$RELOCALIZATION_SETUP" ]] || fatal "relocalizer Jazzy setup not found: $RELOCALIZATION_SETUP"
fi
[[ -f "$PLAN_FILE" ]] || fatal "plan descriptor not found: $PLAN_FILE"
[[ -f "$SURFACE_FILE" ]] || fatal "surface map not found: $SURFACE_FILE"
if [[ -n "$FILTER_CONFIG" ]]; then
  [[ -f "$FILTER_CONFIG" ]] || fatal "filter parameters not found: $FILTER_CONFIG"
fi
command -v setsid >/dev/null 2>&1 || fatal "setsid is required"
command -v timeout >/dev/null 2>&1 || fatal "timeout is required"

if [[ -z "$LOG_DIR" ]]; then
  LOG_DIR="$PWD/.pipeline/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$LOG_DIR"
LOG_DIR="$(cd "$LOG_DIR" && pwd)"

# Keep ROS logs and the ROS home beside the pipeline logs.
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
if [[ -n "$NAV_SETUP" ]]; then
  source "$NAV_SETUP"
fi
if [[ -n "$RELOCALIZATION_SETUP" ]]; then
  source "$RELOCALIZATION_SETUP"
fi
set -u
[[ "$ROS_DISTRO" == jazzy && ":${AMENT_PREFIX_PATH:-}:${CMAKE_PREFIX_PATH:-}:" != *":/opt/ros/humble:"* ]] || \
  fatal "the selected setup files must load Jazzy without a Humble underlay"

for pkg in livox_ros_driver2 fast_lio relocalizer; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fatal "ROS package not found: $pkg"
done
if [[ "$START_FILTER" == "1" ]]; then
  ros2 pkg prefix bw_fastlio_nav2 >/dev/null 2>&1 || \
    fatal "ROS package not found: bw_fastlio_nav2 (use --no-filter to skip it)"
fi

FASTLIO_PREFIX="$(ros2 pkg prefix fast_lio)"
FASTLIO_PARAMS="${FASTLIO_PARAMS:-$FASTLIO_PREFIX/share/fast_lio/config/mid360.yaml}"
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
if [[ -n "$FILTER_CONFIG" ]]; then
  FILTER_PARAMS=(--params-file "$FILTER_CONFIG" -p "input_topic:=$SWEEP_TOPIC")
fi

printf '== live relocalization pipeline ==\n'
printf '  upstream setup: %s\n' "${NAV_SETUP:-already sourced environment}"
printf '  relocalizer:    %s\n' "${RELOCALIZATION_SETUP:-already sourced environment}"
printf '  plan:           %s\n' "$PLAN_FILE"
printf '  surface:        %s\n' "$SURFACE_FILE"
printf '  cloud topic:    %s\n' "$SWEEP_TOPIC"
printf '  odom frame:     %s\n' "$ODOM_FRAME"
printf '  logs:           %s\n\n' "$LOG_DIR"

printf '[1/5] Livox MID360 driver\n'
launch_group lidar_driver ros2 launch livox_ros_driver2 "$LIVOX_LAUNCH"
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
  -p pcd_save.pcd_save_en:=false \
  -r "/cloud_registered_body:=$SWEEP_TOPIC"
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
  printf '       starting %s -> base_footprint static TF\n' "$BODY_FRAME"
  launch_group base_tf ros2 run tf2_ros static_transform_publisher \
    --x "${BASE_TRANSFORM[1]}" --y "${BASE_TRANSFORM[2]}" --z "${BASE_TRANSFORM[3]}" \
    --roll "${BASE_TRANSFORM[4]}" --pitch "${BASE_TRANSFORM[5]}" --yaw "${BASE_TRANSFORM[6]}" \
    --frame-id "$BODY_FRAME" --child-frame-id base_footprint
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
printf '             publishing %s -> %s, FAST-LIO owns %s -> %s\n' \
  "$WORLD_FRAME" "$ODOM_FRAME" "$ODOM_FRAME" "$BODY_FRAME"
printf '             status: ros2 topic echo /relocalizer/status\n'
printf '             live log: %s\n\n' "$LOG_DIR/relocalizer.log"

setsid --wait ros2 launch relocalizer relocalizer.launch.py \
  plan_descriptor:="$PLAN_FILE" \
  surface_map:="$SURFACE_FILE" \
  sweep_topic:="$SWEEP_TOPIC" \
  body_frame:="$BODY_FRAME" \
  world_frame:="$WORLD_FRAME" \
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
