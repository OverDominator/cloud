#!/usr/bin/env bash
set -euo pipefail

session="fire_nav"
if ! tmux has-session -t "$session" 2>/dev/null; then
    echo "No fire_nav session exists" >&2
    exit 1
fi

tmux kill-window -t "$session:cmu_stack" 2>/dev/null || true
tmux new-window -t "$session" -n cmu_stack \
    "bash -lc 'source /opt/ros/noetic/setup.bash; source /root/autonomous_exploration_development_environment/devel/setup.bash; export ROS_PACKAGE_PATH=/root/catkin_ws/src/fire_perception:\${ROS_PACKAGE_PATH}; rosparam set /use_sim_time false; exec roslaunch /root/catkin_ws/src/fire_perception/launch/system_unity_fire.launch'"
