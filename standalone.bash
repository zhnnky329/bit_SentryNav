#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" != "--isolated" ]]; then
  exec env -i HOME="$HOME" USER="${USER:-}" PATH="/usr/local/bin:/usr/bin:/bin" \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" TERM="${TERM:-xterm}" \
    LANG="C.UTF-8" ROS_DOMAIN_ID="${BIT_ROS_DOMAIN_ID:-73}" \
    bash --noprofile --norc "$0" --isolated "$@"
fi
shift
set -u
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
command="${1:-help}"
case "$command" in
  help|-h|--help)
    cat <<'USAGE'
Usage: bash standalone.bash COMMAND [arguments]
  check                      Check the Jazzy environment and independent overlay
  build                      Build independent perception and navigation packages
  perception [launch args]   Single MID360, defaults to replay mode (no driver)
  bag /absolute/bag/path     Replay only /livox/lidar and /livox/imu with /clock
  navigation [launch args]   Run navigation; CAN disabled unless enable_can:=true in live mode
  shell                      Open a shell with only the independent overlay

Navigation uses the bundled map by default; map:=... pcd:=... may override it.
Relocalizer localizes once after startup and latches that map->odom result.
ROS_DOMAIN_ID defaults to 73; override with BIT_ROS_DOMAIN_ID before this script.
Build/install/log directories have the _standalone suffix. CAN is disabled by default.
USAGE
    exit 0 ;;
  check|build|perception|bag|navigation|shell) shift ;;
  *) echo "Unknown command: $command" >&2; exit 2 ;;
esac

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required inside Ubuntu 24.04 with ROS 2 Jazzy." >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ "$command" == build ]]; then
  packages=(bit_standalone navi2 point_lio relocalizer minco_planner minco_controller)
  exec colcon --log-base log_standalone build \
    --build-base build_standalone --install-base install_standalone \
    --symlink-install --executor sequential --packages-up-to "${packages[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
fi

if [[ ! -f install_standalone/local_setup.bash ]]; then
  echo "Missing install_standalone/local_setup.bash; the independent workspace has not been built." >&2
  exit 1
fi
set +u
source install_standalone/local_setup.bash
set -u

case "$command" in
  check)
    for package in bit_standalone livox_ros_driver2 point_lio navi2 minco_planner minco_controller rog_map relocalizer; do
      prefix="$(ros2 pkg prefix "$package")"
      case "$prefix" in
        "$workspace/install_standalone/"*) echo "$package: $prefix" ;;
        *) echo "$package resolved outside install_standalone: $prefix" >&2; exit 1 ;;
      esac
    done
    echo "Environment check passed (ROS_DOMAIN_ID=$ROS_DOMAIN_ID); no nodes were launched." ;;
  perception) exec ros2 launch bit_standalone perception.launch.py "$@" ;;
  bag)
    [[ $# == 1 ]] || { echo "Usage: bash standalone.bash bag /absolute/bag/path" >&2; exit 2; }
    exec ros2 bag play "$1" --clock --topics /livox/lidar /livox/imu ;;
  navigation) exec ros2 launch bit_standalone navigation.launch.py "$@" ;;
  shell) exec bash --noprofile --norc ;;
esac
