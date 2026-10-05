#!/usr/bin/env bash
# Local reconstruction helper; does not fetch or grant redistribution of dependencies.
set -eo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo 'Usage: build_local_snapshot.sh SNAPSHOT NEW_ABSOLUTE_WORKSPACE [--check]' >&2
    exit 2
fi
snapshot=$(realpath -e -- "$1")
workspace=$2
mode=${3:-build}
[[ "$mode" == build || "$mode" == --check ]] || { echo 'Unknown mode' >&2; exit 2; }
[[ "$workspace" == /* && "$workspace" != / && ! -e "$workspace" ]] || {
    echo 'Destination must be a new absolute directory; existing workspaces are never overwritten.' >&2; exit 2;
}
workspace=$(realpath -m -- "$workspace")
[[ ! -e "$workspace" && "$workspace" != / ]] || exit 2
release=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
sources=("$release/ros/fire_perception" "$snapshot/ros_tcp_endpoint")
for package in far_planner visibility_graph_msg graph_decoder boundary_handler goalpoint_rviz_plugin teleop_rviz_plugin; do
    sources+=("$snapshot/far_planner/src/$package")
done
for package in local_planner terrain_analysis terrain_analysis_ext sensor_scan_generation vehicle_simulator; do
    sources+=("$snapshot/exploration_environment_src/$package")
done
for package in joy ps3joy; do
    sources+=("$snapshot/exploration_environment_src/joystick_drivers/$package")
done
for source in "${sources[@]}"; do
    [[ -f "$source/package.xml" ]] || { echo "Missing package: $source" >&2; exit 1; }
done
[[ -f /opt/ros/noetic/setup.bash ]] || { echo 'ROS Noetic setup missing' >&2; exit 1; }
echo "Source layout checked: ${#sources[@]} packages. Destination: $workspace"
if [[ "$mode" == --check ]]; then
    echo 'No files copied, packages installed, or processes launched.'; exit 0
fi
# Prevent accidental original-project overlays in the new build.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH CATKIN_PREFIX_PATH
source /opt/ros/noetic/setup.bash
mkdir -- "$workspace"
mkdir -- "$workspace/src"
for source in "${sources[@]}"; do cp -a -- "$source" "$workspace/src/"; done
cd -- "$workspace"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 2>&1 | tee build.log
echo 'Build finished. Use the same devel/setup.bash for FIRE_ROS_SETUP and FIRE_CMU_SETUP.'
echo 'This does not install OS/Python dependencies, download model weights, or start ROS.'
