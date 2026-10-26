#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble: download/extract missing dependencies locally.
set -euo pipefail
GO2_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_DEPS="$GO2_WS/third_party/ros_deps"
mkdir -p "$GO2_DEPS/debs" "$GO2_DEPS/root"
apt-get -s --no-install-recommends install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox > "$GO2_DEPS/apt-plan.txt"
awk '/^Inst / {print $2 "=" substr($3,2)}' "$GO2_DEPS/apt-plan.txt" > "$GO2_DEPS/packages.txt"
cd "$GO2_DEPS/debs"
if [[ -s "$GO2_DEPS/packages.txt" ]]; then
  xargs -a "$GO2_DEPS/packages.txt" apt-get download
fi
find . -name '*.deb' -print0 | xargs -0 -r -I{} dpkg-deb -x '{}' "$GO2_DEPS/root"
cmake "-DGO2_DEPS_ROOT=$GO2_DEPS/root" -P "$GO2_WS/scripts/relocate_ros_deps.cmake"
: > "$GO2_DEPS/versions.tsv"
for GO2_DEB in *.deb; do
  [[ -f "$GO2_DEB" ]] || continue
  dpkg-deb -f "$GO2_DEB" Package Version >> "$GO2_DEPS/versions.tsv"
done
echo "Dependencies ready. Run: source scripts/ros_env.bash"
