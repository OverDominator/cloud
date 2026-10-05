#!/usr/bin/env python3
import sys

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/terrain_map_ext"
    rospy.init_node("inspect_pointcloud", anonymous=True)
    message = rospy.wait_for_message(topic, PointCloud2, timeout=10.0)
    points = [
        tuple(float(value) for value in point)
        for point in point_cloud2.read_points(
            message, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
    ]
    values = [point[3] for point in points]

    if not values:
        print(f"topic={topic} points=0")
        return

    values.sort()
    count = len(values)

    def percentile(fraction):
        return values[min(count - 1, int((count - 1) * fraction))]

    print(f"topic={topic} points={count}")
    odometry = rospy.wait_for_message("/state_estimation", Odometry, timeout=10.0)
    robot = odometry.pose.pose.position
    print(f"robot x={robot.x:.3f} y={robot.y:.3f} z={robot.z:.3f}")
    print(
        f"cloud x=[{min(p[0] for p in points):.3f}, {max(p[0] for p in points):.3f}] "
        f"y=[{min(p[1] for p in points):.3f}, {max(p[1] for p in points):.3f}] "
        f"z=[{min(p[2] for p in points):.3f}, {max(p[2] for p in points):.3f}]"
    )
    for radius in (5.0, 10.0, 15.0):
        nearby = [
            point
            for point in points
            if (point[0] - robot.x) ** 2 + (point[1] - robot.y) ** 2
            <= radius**2
            and abs(point[2] - robot.z) <= 1.85
        ]
        nearby_obstacles = sum(point[3] >= 0.1 for point in nearby)
        print(
            f"crop radius={radius:.0f} z_tol=1.85: "
            f"points={len(nearby)} obs>=0.10={nearby_obstacles}"
        )
    print(
        "intensity "
        f"min={values[0]:.6f} p50={percentile(0.50):.6f} "
        f"p90={percentile(0.90):.6f} p95={percentile(0.95):.6f} "
        f"p99={percentile(0.99):.6f} max={values[-1]:.6f}"
    )
    for threshold in (0.02, 0.05, 0.10, 0.15, 0.20):
        obstacle_count = sum(value >= threshold for value in values)
        print(
            f">={threshold:.2f}: {obstacle_count} "
            f"({100.0 * obstacle_count / count:.2f}%)"
        )


if __name__ == "__main__":
    main()
