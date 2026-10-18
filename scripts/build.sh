#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
mode="${1:-base}"
if [[ "$mode" != base && "$mode" != examples && "$mode" != armor ]]; then
  echo 'Usage: bash scripts/build.sh [base|examples|armor]' >&2; exit 2
fi
mkdir -p "$KIT_ROOT/.ros_ws"
cd "$KIT_ROOT/.ros_ws"
packages=("$KIT_ROOT/simulator/rm_interfaces")
if [[ "$mode" == examples ]]; then
  packages+=("$KIT_ROOT/examples/simplified_auto_aim/ros2/auto_aim_ros2")
fi
if [[ "$mode" == examples || "$mode" == armor ]]; then
  packages+=(
    "$KIT_ROOT/examples/armor_aim/src/prm_vision/vision_msgs"
    "$KIT_ROOT/examples/armor_aim/src/prm_vision/opencv_armor_detector"
    "$KIT_ROOT/examples/armor_aim/src/prm_vision/pose_estimator"
    "$KIT_ROOT/examples/armor_aim/src/prm_control/aim_output"
    "$KIT_ROOT/examples/armor_aim/src/prm_launch"
  )
fi
colcon build --base-paths "${packages[@]}" --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$KIT_ROOT/.ros_ws/install/setup.bash"
cd "$KIT_ROOT/simulator"
cargo build --locked --release --no-default-features --features ros2 --bin daedalus
