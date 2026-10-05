#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
set -u

timeout 10 rosservice call /experiment_logger/stop
