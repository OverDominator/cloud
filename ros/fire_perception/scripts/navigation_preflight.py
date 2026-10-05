#!/usr/bin/env python3
"""Read-only startup diagnostic; does not start, reset or move the robot."""
import sys

import rospy
import rosgraph
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, UInt32


def main():
    rospy.init_node("navigation_preflight", anonymous=True)
    publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
    by_topic = dict(publishers)
    failures = 0
    for topic in ("/state_estimation", "/cmd_vel", "/cmd_vel_raw", "/thermal/image_raw"):
        nodes = by_topic.get(topic, [])
        ok = len(nodes) == 1
        failures += not ok
        print("{} {} publishers={}".format("OK" if ok else "ERROR", topic, nodes))
    nodes = {node for _, entries in publishers for node in entries}
    print("Local planner: {}".format(
        "move_base/TEB (verify rosparam /move_base/base_local_planner)"
        if "/move_base" in nodes else "CMU/other; move_base is not publishing"))
    for topic, kind in (("/state_estimation", Odometry),
                        ("/thermal/image_raw", Image),
                        ("/thermal/camera_info", CameraInfo),
                        ("/victim_mission/state", String),
                        ("/local_recovery/state", String),
                        ("/unity_motion_guard/status", String),
                        ("/victim_detection/count", UInt32)):
        try:
            msg = rospy.wait_for_message(topic, kind, timeout=2.0)
            if hasattr(msg, "header"):
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                print("{} age={:.2f}s".format(topic, age))
                if age > 1.5 or age < -0.25:
                    failures += 1
            else:
                print("{} {}".format(topic, msg.data))
        except rospy.ROSException:
            failures += 1
            print("ERROR {} no message in 2s".format(topic))
    print("Preflight {} ({} errors). This is not a rescue-success test.".format(
        "PASS" if failures == 0 else "FAIL", failures))
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
