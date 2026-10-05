#!/usr/bin/env bash
set -euo pipefail

SESSION="fire_nav"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Fire navigation is already running in tmux session: $SESSION"
    echo "Use: tmux attach -t $SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n roscore \
    "bash -lc 'source /opt/ros/noetic/setup.bash; exec roscore'"

tmux new-window -t "$SESSION" -n endpoint \
    "bash -lc 'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; sleep 3; exec roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10001'"

tmux new-window -t "$SESSION" -n cmu_stack \
    "bash -lc 'source /opt/ros/noetic/setup.bash; source /root/autonomous_exploration_development_environment/devel/setup.bash; export ROS_PACKAGE_PATH=/root/catkin_ws/src/fire_perception:\${ROS_PACKAGE_PATH}; sleep 5; rosparam set /use_sim_time false; exec roslaunch /root/catkin_ws/src/fire_perception/launch/system_unity_fire.launch'"

tmux new-window -t "$SESSION" -n far_planner \
    "bash -lc 'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; sleep 8; rosparam set /use_sim_time false; export DISABLE_ROS1_EOL_WARNINGS=1; exec roslaunch fire_perception far_unity_fire.launch'"

tmux new-window -t "$SESSION" -n fire_perception \
    "bash -lc 'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; sleep 8; exec roslaunch fire_perception fire_color_detector.launch'"

tmux select-window -t "$SESSION:roscore"

echo "Fire navigation startup launched."
echo "1. Start Unity Play after the endpoint window reports port 10001 is listening."
echo "2. View all processes with: tmux attach -t $SESSION"
echo "3. Leave tmux without stopping ROS: Ctrl+B, then D"
echo "4. Stop managed processes with: ~/stop_fire_nav.sh"
