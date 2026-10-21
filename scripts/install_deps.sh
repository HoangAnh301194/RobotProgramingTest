#!/usr/bin/env bash
set -euo pipefail
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: Source /opt/ros/humble/setup.bash before running." >&2
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-moveit ros-humble-moveit-configs-utils \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager ros-humble-ros2-controllers \
  ros-humble-position-controllers ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-tf2-ros
if [[ ! -d "$WORKSPACE_SRC/IFRA_LinkAttacher" ]]; then
  git clone --branch humble --depth 1 \
    https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git \
    "$WORKSPACE_SRC/IFRA_LinkAttacher"
fi
echo "Dependencies installed. Now run: cd $(dirname "$WORKSPACE_SRC") && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
