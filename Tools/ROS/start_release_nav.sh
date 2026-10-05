#!/usr/bin/env bash
set -eo pipefail
# Specify both built workspace setup.bash files; --check starts no processes.
: "${FIRE_ROS_SETUP:?Set FIRE_ROS_SETUP to your ROS workspace setup.bash}"
: "${FIRE_CMU_SETUP:?Set FIRE_CMU_SETUP to your modified CMU workspace setup.bash}"
ros_setup=${FIRE_SYSTEM_SETUP:-/opt/ros/noetic/setup.bash}
session=fire_nav
for setup in "$ros_setup" "$FIRE_CMU_SETUP" "$FIRE_ROS_SETUP"; do
    [[ -f "$setup" ]] || { echo "Missing setup: $setup" >&2; exit 1; }
done
source "$ros_setup"
source "$FIRE_CMU_SETUP"
source "$FIRE_ROS_SETUP"
for package in fire_perception vehicle_simulator far_planner ros_tcp_endpoint; do
    rospack find "$package" || exit 1
done
stack_launch="$(rospack find vehicle_simulator)/launch/system_unity.launch"
[[ -f "$stack_launch" ]] || { echo 'Missing custom Unity stack launch' >&2; exit 1; }
if [[ "${1:-}" == '--check' ]]; then
    echo 'Package/path checks passed; no ROS nodes started. Runtime dependencies are not fully validated.'
    exit 0
fi
[[ $# == 0 ]] || { echo 'Usage: start_release_nav.sh [--check]' >&2; exit 2; }
command -v tmux >/dev/null
if tmux has-session -t "$session" 2>/dev/null; then
    echo "Session $session already exists; nothing started."
    exit 0
fi
printf -v prefix 'source %q && source %q && source %q && ' "$ros_setup" "$FIRE_CMU_SETUP" "$FIRE_ROS_SETUP"
window() {
    local name=$1 command=$2 quoted
    printf -v quoted 'bash -c %q' "$prefix$command"
    if [[ "$name" == roscore ]]; then
        tmux new-session -d -s "$session" -n "$name" "$quoted"
    else
        tmux new-window -t "$session" -n "$name" "$quoted"
    fi
}
window roscore 'exec roscore'
window endpoint 'sleep 3; exec roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10001'
window cmu_stack 'sleep 5; rosparam set /use_sim_time false && exec roslaunch fire_perception system_unity_fire.launch'
window far_planner 'sleep 8; rosparam set /use_sim_time false && exec roslaunch fire_perception far_unity_fire.launch'
window fire_perception 'sleep 8; exec roslaunch fire_perception fire_color_detector.launch'
tmux select-window -t "$session:roscore"
echo "Startup requested. Inspect logs: tmux attach -t $session"
echo 'Start Unity only after endpoint and navigation readiness are confirmed.'
echo 'Stop this managed session: tmux kill-session -t fire_nav'
