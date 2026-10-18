#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
if [[ ! -f "$KIT_ROOT/.ros_ws/install/setup.bash" ]]; then
  echo 'Build first: bash scripts/build.sh' >&2; exit 1
fi
cd "$KIT_ROOT/simulator"
# Keep cargo run: Bevy dynamic_linking needs Cargo's runtime library search path.
exec cargo run --locked --release --no-default-features --features ros2 --bin daedalus -- "$@"
