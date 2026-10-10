#!/usr/bin/env bash
# Explicit host installer, Ubuntu 22.04 only. No distro upgrade or shell edits.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/os-release
INSTALL_SCOPE="${1:-all}"
case "$INSTALL_SCOPE" in all|--system-only) ;; *) echo 'Usage: install_host.sh [--system-only]'; exit 2 ;; esac
[ "$ID" = ubuntu ] && [ "$VERSION_ID" = 22.04 ] || { echo 'Requires Ubuntu 22.04'; exit 1; }
SUDO=()
[ "$EUID" -eq 0 ] || SUDO=(sudo)
for pkg in ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gzgarden; do
  if dpkg-query -W -f='${db:Status-Status}' "$pkg" 2>/dev/null | grep -qx installed; then
    echo "Conflicting $pkg installed; resolve explicitly before continuing."; exit 1
  fi
done
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl ca-certificates gnupg software-properties-common
"${SUDO[@]}" add-apt-repository -y universe
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | "${SUDO[@]}" tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | "${SUDO[@]}" tee /usr/share/keyrings/gazebo-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' | "${SUDO[@]}" tee /etc/apt/sources.list.d/uav-lab-ros.list >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' | "${SUDO[@]}" tee /etc/apt/sources.list.d/uav-lab-gazebo.list >/dev/null
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y build-essential cmake ninja-build git ccache pkg-config \
  python3-pip python3-dev python3-colcon-common-extensions python3-pytest python3-empy \
  python3-jinja2 python3-numpy python3-scipy python3-toml python3-yaml python3-packaging python3-jsonschema python3-serial python3-psutil python3-opencv \
  nlohmann-json3-dev libeigen3-dev libxml2-dev libssl-dev libtinyxml2-dev libspdlog-dev libasio-dev libfoonathan-memory-dev \
  libboost-all-dev libtbb-dev ros-humble-gtsam ros-humble-ament-cmake-auto \
  ros-humble-cv-bridge ros-humble-image-transport \
  gz-harmonic ros-humble-ros-base ros-humble-ros-gzharmonic ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-diagnostic-msgs ros-humble-nav-msgs ros-humble-geometry-msgs \
  ros-humble-std-srvs ros-humble-rosidl-default-generators ros-humble-sensor-msgs \
  ros-humble-rosbag2 ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins
/usr/bin/python3 -m pip install --user -r "$ROOT/dependencies/build-requirements.txt"
/usr/bin/python3 -m pip install --user -r "$ROOT/dependencies/observation-requirements.txt"
[ "$INSTALL_SCOPE" != --system-only ] || exit 0
/usr/bin/python3 "$ROOT/scripts/bootstrap.py" --jobs 2
"$ROOT/scripts/build.sh"
/usr/bin/python3 "$ROOT/scripts/doctor.py" --runtime
