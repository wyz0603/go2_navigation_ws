#!/usr/bin/env bash
# Source this file from the workspace root to load ROS and the built overlay.

go2_workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy" >&2
  return 1
fi
source /opt/ros/jazzy/setup.bash || return 1

if [[ -f "$go2_workspace_dir/install/setup.bash" ]]; then
  source "$go2_workspace_dir/install/setup.bash" || return 1
fi
