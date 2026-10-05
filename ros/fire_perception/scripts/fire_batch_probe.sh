#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
set -u

case "${1:-}" in
    state_ready)
        timeout 4 rostopic echo -n 1 /state_estimation >/dev/null
        ;;
    recording)
        timeout 4 rostopic echo -n 1 /experiment_logger/status | grep -q 'recording:'
        ;;
    mission)
        timeout 4 rostopic echo -n 1 /victim_mission/state
        ;;
    *)
        echo "usage: $0 {state_ready|recording|mission}" >&2
        exit 2
        ;;
esac
