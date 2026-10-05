#!/usr/bin/env python3
import json
import math
import os
import statistics
import sys

import rosbag


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int((len(ordered) - 1) * fraction))
    return ordered[index]


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_navigation_bag.py PATH_TO_BAG")

    bag_path = os.path.abspath(sys.argv[1])
    positions = []
    commands = []
    planning_times = []
    goals = []
    waypoint_count = 0
    collision_count = 0
    reached_goal = False

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, bag_time in bag.read_messages():
            timestamp = bag_time.to_sec()
            if topic == "/state_estimation":
                point = message.pose.pose.position
                positions.append((timestamp, point.x, point.y, point.z))
            elif topic == "/cmd_vel":
                commands.append(
                    (
                        timestamp,
                        float(message.twist.linear.x),
                        float(message.twist.angular.z),
                    )
                )
            elif topic == "/planning_time":
                planning_times.append(float(message.data))
            elif topic == "/goal_point":
                point = message.point
                candidate = (round(point.x, 3), round(point.y, 3), round(point.z, 3))
                if not goals or goals[-1] != candidate:
                    goals.append(candidate)
            elif topic == "/way_point":
                waypoint_count += 1
            elif topic == "/collision_count":
                collision_count = max(collision_count, int(message.data))
            elif topic == "/far_reach_goal_status":
                reached_goal = reached_goal or bool(message.data)

    path_length = 0.0
    for previous, current in zip(positions, positions[1:]):
        path_length += math.hypot(current[1] - previous[1], current[2] - previous[2])

    duration = positions[-1][0] - positions[0][0] if len(positions) > 1 else 0.0
    linear_speeds = [abs(command[1]) for command in commands]
    angular_speeds = [abs(command[2]) for command in commands]

    metrics = {
        "bag": bag_path,
        "duration_s": round(duration, 3),
        "path_length_m": round(path_length, 3),
        "state_samples": len(positions),
        "state_rate_hz": round(len(positions) / duration, 3) if duration > 0 else 0.0,
        "goal_count": len(goals),
        "goals": goals,
        "waypoint_messages": waypoint_count,
        "reached_goal": reached_goal,
        "collision_count": collision_count,
        "mean_abs_linear_speed": round(statistics.mean(linear_speeds), 4) if linear_speeds else 0.0,
        "max_abs_linear_speed": round(max(linear_speeds), 4) if linear_speeds else 0.0,
        "max_abs_angular_speed": round(max(angular_speeds), 4) if angular_speeds else 0.0,
        "mean_planning_time": round(statistics.mean(planning_times), 6) if planning_times else None,
        "p95_planning_time": round(percentile(planning_times, 0.95), 6) if planning_times else None,
    }

    output_path = os.path.join(os.path.dirname(bag_path), "metrics.json")
    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(metrics, output_file, ensure_ascii=False, indent=2)

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print("metrics written to: {}".format(output_path))


if __name__ == "__main__":
    main()
