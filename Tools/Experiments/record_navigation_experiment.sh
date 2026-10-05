#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SCENARIO="${1:-baseline_navigation}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$PROJECT_ROOT/ExperimentData/${SCENARIO}/${RUN_ID}"
mkdir -p "$RUN_DIR"

TOPICS=(
    /odom
    /state_estimation
    /goal_point
    /way_point
    /cmd_vel
    /speed
    /stop
    /collision_count
    /far_reach_goal_status
    /planning_time
    /runtime
    /far_traverse_time
    /tf
    /tf_static
)

if [[ "${2:-}" == "--with-cloud" ]]; then
    TOPICS+=(/registered_scan /terrain_map /terrain_map_ext)
fi

{
    echo "scenario=$SCENARIO"
    echo "run_id=$RUN_ID"
    echo "started_at=$(date --iso-8601=seconds)"
    echo "hostname=$(hostname)"
    echo "use_sim_time=$(rosparam get /use_sim_time 2>/dev/null || echo unknown)"
    echo "with_cloud=$([[ "${2:-}" == "--with-cloud" ]] && echo true || echo false)"
} > "$RUN_DIR/metadata.txt"

echo "Recording experiment: $SCENARIO"
echo "Output: $RUN_DIR/navigation.bag"
echo "Press Ctrl+C after the robot reaches the goal."

exec rosbag record \
    --buffsize=256 \
    --chunksize=768 \
    -O "$RUN_DIR/navigation.bag" \
    "${TOPICS[@]}"
