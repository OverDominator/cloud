#!/usr/bin/env python3

import math
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker


class FarMoveBaseBridge:
    """Starts TEB only after FAR has published a path for the current goal."""

    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_point")
        self.recovery_goal_topic = rospy.get_param(
            "~recovery_goal_topic", "/recovery_test/goal"
        )
        self.path_topic = rospy.get_param("~far_path_topic", "/viz_path_topic")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.teb_command_topic = rospy.get_param(
            "~teb_command_topic", "/cmd_vel_teb"
        )
        self.raw_command_topic = rospy.get_param(
            "~raw_command_topic", "/cmd_vel_raw"
        )
        self.goal_change_distance = float(
            rospy.get_param("~goal_change_distance", 0.35)
        )
        self.path_endpoint_tolerance = float(
            rospy.get_param("~path_endpoint_tolerance", 8.0)
        )

        self.lock = threading.RLock()
        self.pending_goal = None
        self.last_sent_goal = None
        self.latest_goal = None
        self.latest_path = None
        self.client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        self.raw_command_publisher = rospy.Publisher(
            self.raw_command_topic, TwistStamped, queue_size=5
        )

        rospy.Subscriber(
            self.goal_topic, PointStamped, self.goal_callback, queue_size=5
        )
        # Recovery benchmark scenes do not run the victim mission manager.
        # Feed their fixed goal into the same FAR -> local execution chain;
        # Keep goal execution on the FAR planning chain.
        if self.recovery_goal_topic and self.recovery_goal_topic != self.goal_topic:
            rospy.Subscriber(
                self.recovery_goal_topic,
                PointStamped,
                self.goal_callback,
                queue_size=5,
            )
        rospy.Subscriber(
            self.path_topic, Marker, self.path_callback, queue_size=5
        )
        rospy.Subscriber(
            self.teb_command_topic, Twist, self.command_callback, queue_size=5
        )
        self.server_timer = rospy.Timer(
            rospy.Duration(0.5), self.server_timer_callback
        )
        rospy.loginfo(
            "FAR-to-move_base bridge: %s + %s -> %s",
            self.goal_topic,
            self.path_topic,
            self.move_base_action,
        )

    def command_callback(self, message):
        stamped = TwistStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = "vehicle"
        stamped.twist = message
        self.raw_command_publisher.publish(stamped)

    @staticmethod
    def distance(first, second):
        return math.hypot(first.x - second.x, first.y - second.y)

    def server_timer_callback(self, _event):
        if not self.client.wait_for_server(rospy.Duration(0.01)):
            rospy.logwarn_throttle(3.0, "Waiting for move_base action server")
            return
        self.try_send_goal()

    def goal_callback(self, message):
        with self.lock:
            self.latest_goal = message
            if (
                self.last_sent_goal is not None
                and self.distance(
                    self.last_sent_goal.point, message.point
                ) <= self.goal_change_distance
            ):
                return
            self.pending_goal = message
        self.try_send_goal()

    def path_callback(self, message):
        if (
            message.ns != "global_path"
            or message.type != Marker.LINE_STRIP
            or message.action in (Marker.DELETE, Marker.DELETEALL)
            or len(message.points) < 2
        ):
            return
        with self.lock:
            self.latest_path = message
        self.try_send_goal()

    def path_matches_goal(self, path, goal):
        first_distance = self.distance(path.points[0], goal.point)
        last_distance = self.distance(path.points[-1], goal.point)
        return min(first_distance, last_distance) <= self.path_endpoint_tolerance

    def try_send_goal(self):
        if not self.client.wait_for_server(rospy.Duration(0.01)):
            return
        with self.lock:
            goal_point = self.pending_goal
            path = self.latest_path
            if goal_point is None or path is None:
                return
            if not self.path_matches_goal(path, goal_point):
                rospy.logwarn_throttle(
                    2.0, "Waiting for FAR path matching the current goal"
                )
                return

            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.stamp = rospy.Time.now()
            move_goal.target_pose.header.frame_id = (
                goal_point.header.frame_id or "map"
            )
            move_goal.target_pose.pose.position.x = goal_point.point.x
            move_goal.target_pose.pose.position.y = goal_point.point.y
            move_goal.target_pose.pose.position.z = goal_point.point.z

            path_points = list(path.points)
            if self.distance(path_points[0], goal_point.point) < self.distance(
                path_points[-1], goal_point.point
            ):
                path_points.reverse()
            previous = path_points[-2]
            final = path_points[-1]
            yaw = math.atan2(final.y - previous.y, final.x - previous.x)
            move_goal.target_pose.pose.orientation.z = math.sin(yaw * 0.5)
            move_goal.target_pose.pose.orientation.w = math.cos(yaw * 0.5)

            self.pending_goal = None
            self.last_sent_goal = goal_point

        self.client.send_goal(move_goal, done_cb=self.done_callback)
        rospy.loginfo(
            "Sent FAR-backed TEB goal (%.2f, %.2f)",
            goal_point.point.x,
            goal_point.point.y,
        )

    def done_callback(self, state, _result):
        if state not in (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.LOST,
        ):
            return
        with self.lock:
            rospy.logwarn("move_base ended with state %d; waiting for FAR retry", state)
            self.last_sent_goal = None
            self.pending_goal = self.latest_goal


def main():
    rospy.init_node("far_move_base_bridge")
    FarMoveBaseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
