#!/usr/bin/env python3
"""Normal-only FAR command adapter; safety vetoes stay downstream."""
import threading
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt32


def clamp(value, low, high):
    return max(low, min(high, value))


class FarCommandAdapter:
    def __init__(self):
        self.lock = threading.RLock()
        if rospy.get_param("~enable_active_recovery", False):
            raise RuntimeError("Active recovery is not included in this release")
        for key, default in [('control_rate', 20.0), ('normal_max_forward_speed', .90),
                             ('normal_max_reverse_speed', .55), ('normal_max_turn_rate', .90),
                             ('normal_high_speed_turn_threshold', .75),
                             ('normal_high_speed_max_turn_rate', .45)]:
            setattr(self, key, rospy.get_param('~' + key, default))
        self.raw_timeout = rospy.get_param('~raw_command_timeout', .6)
        self.odom_timeout = rospy.get_param('~odom_timeout', .8)
        self.raw_command = None
        self.odom = None
        self.raw_command_time = rospy.Time(0)
        self.odom_time = rospy.Time(0)
        self.mission_active = True
        raw_topic = rospy.get_param('~raw_command_topic', '/cmd_vel_raw')
        out_topic = rospy.get_param('~output_command_topic', '/cmd_vel')
        self.cmd_pub = rospy.Publisher(out_topic, TwistStamped, queue_size=5)
        self.replan_pub = rospy.Publisher('/local_recovery/replan_required', Bool, queue_size=1, latch=True)
        self.replan_pub.publish(Bool(data=False))
        # Preserve the normal-mode status contract for mission and logging nodes.
        self.status_publishers = []
        for topic, kind, value in [('state', String, 'normal'), ('active', Bool, False),
                                   ('count', UInt32, 0), ('failure', String, '')]:
            publisher = rospy.Publisher('/local_recovery/' + topic, kind, queue_size=1, latch=True)
            publisher.publish(kind(data=value))
            self.status_publishers.append(publisher)
        rospy.Subscriber(raw_topic, TwistStamped, self.raw_callback, queue_size=5)
        rospy.Subscriber('/state_estimation', Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber('/victim_mission/active', Bool, self.mission_active_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control)

    def raw_callback(self, msg):
        with self.lock:
            self.raw_command = msg
            self.raw_command_time = rospy.Time.now()

    def odom_callback(self, msg):
        with self.lock:
            self.odom = msg
            self.odom_time = rospy.Time.now()

    def mission_active_callback(self, msg):
        with self.lock:
            self.mission_active = bool(msg.data)
            if not self.mission_active:
                self.raw_command = None
                self.replan_pub.publish(Bool(data=False))
                self.cmd_pub.publish(self.make_command(0.0, 0.0, rospy.Time.now()))

    def make_command(self, linear, angular, now):
        msg = TwistStamped()
        msg.header.stamp = now
        msg.header.frame_id = 'vehicle'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        return msg

    def raw_copy(self, now):
        if self.raw_command is None or now - self.raw_command_time > rospy.Duration(self.raw_timeout):
            return self.make_command(0.0, 0.0, now)
        msg = TwistStamped()
        msg.header.seq = self.raw_command.header.seq
        msg.header.frame_id = self.raw_command.header.frame_id or 'vehicle'
        msg.header.stamp = now
        for vector in ('linear', 'angular'):
            for axis in ('x', 'y', 'z'):
                setattr(getattr(msg.twist, vector), axis, getattr(getattr(self.raw_command.twist, vector), axis))
        msg.twist.linear.x = clamp(msg.twist.linear.x, -self.normal_max_reverse_speed, self.normal_max_forward_speed)
        msg.twist.angular.z = clamp(msg.twist.angular.z, -self.normal_max_turn_rate, self.normal_max_turn_rate)
        if msg.twist.linear.x >= self.normal_high_speed_turn_threshold:
            msg.twist.angular.z = clamp(msg.twist.angular.z, -self.normal_high_speed_max_turn_rate, self.normal_high_speed_max_turn_rate)
        return msg

    def control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if not self.mission_active:
                self.cmd_pub.publish(self.make_command(0.0, 0.0, now))
                return
            if self.odom is None or now - self.odom_time > rospy.Duration(self.odom_timeout):
                rospy.logerr_throttle(2.0, 'FAR adapter stopped: stale /state_estimation')
                self.cmd_pub.publish(self.make_command(0.0, 0.0, now))
                return
            self.cmd_pub.publish(self.raw_copy(now))


if __name__ == '__main__':
    rospy.init_node('local_wall_recovery_controller')
    FarCommandAdapter()
    rospy.spin()
