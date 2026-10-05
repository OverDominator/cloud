#!/usr/bin/env python3
"""Motion veto with transport reconnection; never generates recovery motion."""
import math
import threading
import time
import struct
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int8, String, UInt32
from visualization_msgs.msg import Marker, MarkerArray
try:
    from navigation_safety_geometry import swept_rectangle_clear, swept_fire_clear
except (ImportError, AttributeError):
    # catkin_install_python creates a relay wrapper for this helper module;
    # importing that wrapper does not expose its exec() context. Load the
    # source functions explicitly so a fresh install cannot start unprotected.
    import os
    import runpy
    _geometry_path = os.path.join(os.path.dirname(__file__),
                                  'navigation_safety_geometry.py')
    _geometry = runpy.run_path(_geometry_path)
    swept_rectangle_clear = _geometry['swept_rectangle_clear']
    swept_fire_clear = _geometry['swept_fire_clear']


class NavigationSafetyGuard:
    def __init__(self):
        self.lock = threading.RLock()
        self.raw = self.odom = self.scan = None
        self.unity_command = None
        self.unity_status = ('unavailable', None)
        self.physics_diagnostic = ('unavailable', None)
        self.active, self.stop = True, 0
        self.zones = {}
        # FireRescue_Simplified scale-1 chassis, including a conservative
        # allowance for its wheel colliders.  The launch file supplies these
        # explicitly; the defaults keep ad-hoc launches consistent.
        self.half_length = rospy.get_param('~vehicle_length', 1.35) / 2
        self.half_width = rospy.get_param('~vehicle_width', 1.80) / 2
        self.margin = rospy.get_param('~safety_margin', .30)
        self.motion_horizon = max(
            .1, float(rospy.get_param('~motion_check_horizon', .6)))
        # Registered scan is intentionally bandwidth-limited to 1 Hz in
        # Unity.  Allow normal scheduling jitter without treating a healthy
        # stream as stale and stopping the vehicle every other frame.
        self.scan_timeout = max(
            1.0, float(rospy.get_param('~scan_timeout', 1.8)))
        self.in_place_linear_threshold = max(
            0.0, float(rospy.get_param('~in_place_linear_threshold', .08)))
        configured_turn_speeds = rospy.get_param(
            '~safe_turn_retry_speeds', [.55, .40, .25, .15])
        self.safe_turn_retry_speeds = sorted(
            {
                abs(float(speed)) for speed in configured_turn_speeds
                if math.isfinite(float(speed)) and abs(float(speed)) >= .05
            },
            reverse=True,
        )
        self.output = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
        self.reason = rospy.Publisher('/local_recovery/guard_reason', String, queue_size=1, latch=True)
        # This node is the final motion authority in the FAR configuration.
        # Publish its verdict on a dedicated topic as well: the compatibility
        # topic above may also be owned by the optional recovery controller,
        # which makes it unsuitable as an unambiguous replanning trigger.
        self.navigation_reason = rospy.Publisher(
            '/navigation_safety/guard_reason', String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher('/navigation_baseline/sensors_ready', Bool, queue_size=1, latch=True)
        self.diagnostic_pub = rospy.Publisher('/navigation_baseline/safety_diagnostic', String, queue_size=1, latch=True)
        self.last_diagnostic = rospy.Time(0)
        self.compat = []
        for topic, kind, value in [('state', String, 'normal'), ('active', Bool, False),
                                  ('replan_required', Bool, False), ('failure', String, ''),
                                  ('count', UInt32, 0)]:
            pub = rospy.Publisher('/local_recovery/' + topic, kind, queue_size=1, latch=True)
            pub.publish(kind(data=value))
            self.compat.append(pub)
        rospy.Subscriber('/cmd_vel_raw', TwistStamped, lambda m: self.set_value('raw', m), queue_size=1)
        rospy.Subscriber('/state_estimation', Odometry, lambda m: self.set_value('odom', m), queue_size=1)
        self.scan_received_at = time.monotonic()
        self.scan_retry_at = self.scan_received_at
        self.scan_subscriber = self.subscribe_scan()
        self.scan_watchdog = rospy.Timer(rospy.Duration(1.0), self.check_scan_connection)
        rospy.Subscriber('/stop', Int8, lambda m: self.set_value('stop', m.data), queue_size=1)
        rospy.Subscriber('/victim_mission/active', Bool, lambda m: self.set_value('active', m.data), queue_size=1)
        rospy.Subscriber('/fire_detection/keepout_markers', MarkerArray, self.fire_callback, queue_size=1)
        rospy.Subscriber('/unity_motion_guard/applied_cmd', TwistStamped,
                         lambda m: self.set_value('unity_command', m), queue_size=1)
        rospy.Subscriber('/unity_motion_guard/status', String,
                         lambda m: self.set_value('unity_status', (m.data, rospy.Time.now())), queue_size=1)
        rospy.Subscriber('/unity_motion_guard/physics_diagnostic', String,
                         lambda m: self.set_value('physics_diagnostic', (m.data, rospy.Time.now())), queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.05), self.control)

    def set_value(self, name, value):
        with self.lock:
            setattr(self, name, value)

    def scan_callback(self, msg):
        with self.lock:
            self.scan_received_at = time.monotonic()
        if msg.header.frame_id.lstrip('/') != 'map':
            self.set_value('scan', None)
            rospy.logwarn_throttle(5.0, 'Scan rejected: expected map frame, got %s', msg.header.frame_id)
            return
        try:
            if not {'x', 'y', 'z'}.issubset({field.name for field in msg.fields}):
                raise ValueError('missing xyz fields')
            points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        except (ValueError, TypeError, struct.error) as exc:
            self.set_value('scan', None)
            rospy.logwarn_throttle(5.0, 'Scan rejected: %s', str(exc))
            return
        self.set_value('scan', (msg.header.stamp, points))

    def subscribe_scan(self):
        return rospy.Subscriber('/registered_scan', PointCloud2, self.scan_callback,
                                queue_size=1, buff_size=4 * 1024 * 1024)

    def check_scan_connection(self, _event):
        # Retry missing transport data, not old source timestamps. Invalid or
        # stale messages must remain unsafe rather than acquire a fresh stamp.
        now = time.monotonic()
        with self.lock:
            if now - self.scan_received_at < 5.0 or now - self.scan_retry_at < 5.0:
                return
            self.scan_retry_at = now
        # ROS registration may block; never hold the motion-control lock here.
        rospy.logwarn('No scan messages for 5 seconds; reconnecting /registered_scan')
        try:
            if self.scan_subscriber is not None:
                self.scan_subscriber.unregister()
            self.scan_subscriber = self.subscribe_scan()
        except Exception as exc:
            rospy.logerr('Scan reconnect failed; safety remains active: %s', str(exc))

    def fire_callback(self, msg):
        with self.lock:
            for marker in msg.markers:
                if marker.action == Marker.DELETEALL:
                    self.zones.clear()
                elif marker.ns == 'fire_keepout':
                    key = (marker.ns, marker.id)
                    if marker.action == Marker.DELETE:
                        self.zones.pop(key, None)
                    elif marker.type == Marker.CYLINDER and marker.header.frame_id.lstrip('/') == 'map':
                        p = marker.pose.position
                        self.zones[key] = (p.x, p.y, max(marker.scale.x, marker.scale.y)/2)

    def safe_reduced_turn(self, points, pose, requested_v, requested_w):
        """Return the fastest lower in-place turn that passes every guard.

        FAR's local planner commonly requests its maximum angular velocity when
        the first path segment lies behind the robot.  Close to a wall the
        full-horizon sweep of that command can be unsafe even though a slower,
        continuously re-evaluated turn is safe.  Shape only near-zero-linear
        rotation commands; never invent translation or reverse the requested
        turn direction.
        """
        if (
            abs(requested_v) > self.in_place_linear_threshold
            or abs(requested_w) < .05
        ):
            return None
        direction = 1.0 if requested_w > 0.0 else -1.0
        for speed in self.safe_turn_retry_speeds:
            if speed >= abs(requested_w) - 1e-6:
                continue
            candidate_w = direction * speed
            if not swept_rectangle_clear(
                points, 0.0, candidate_w, self.motion_horizon,
                self.half_length, self.half_width, self.margin,
            ):
                continue
            if not swept_fire_clear(
                list(self.zones.values()), pose, 0.0, candidate_w,
                self.motion_horizon,
                math.hypot(self.half_length, self.half_width) + self.margin,
            ):
                continue
            return candidate_w
        return None

    def control(self, _event):
        now = rospy.Time.now()
        output = TwistStamped()
        output.header.stamp, output.header.frame_id = now, 'vehicle'
        reason = 'clear'
        with self.lock:
            odom_age = (now-self.odom.header.stamp).to_sec() if self.odom is not None else float('inf')
            scan_age = (now-self.scan[0]).to_sec() if self.scan is not None else float('inf')
            ready = (0 <= odom_age <= .8 and 0 <= scan_age <= self.scan_timeout
                     and self.odom.header.frame_id.lstrip('/') == 'map'
                     and len(self.scan[1]) > 0)
            self.ready_pub.publish(Bool(data=ready))
            if not self.active or self.stop:
                reason = 'mission_stop'
            elif self.raw is None or not 0 <= (now-self.raw.header.stamp).to_sec() <= .6:
                reason = 'command_stale'
            elif self.odom is None or not 0 <= (now-self.odom.header.stamp).to_sec() <= .8:
                reason = 'odometry_stale'
            elif self.scan is None or not 0 <= (now-self.scan[0]).to_sec() <= self.scan_timeout:
                reason = 'scan_stale'
            elif not ready:
                reason = 'sensor_frame_or_empty_scan'
            else:
                v, w = self.raw.twist.linear.x, self.raw.twist.angular.z
                p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if not all(math.isfinite(x) for x in (v, w, p.x, p.y, p.z, yaw)):
                    reason = 'invalid_motion'
                else:
                    v, w = max(-.55, min(1.12, v)), max(-.9, min(.9, w))
                    c, s = math.cos(yaw), math.sin(yaw)
                    points = [(c*(x-p.x)+s*(y-p.y), -s*(x-p.x)+c*(y-p.y))
                              for x,y,z in self.scan[1]
                              if .18 <= z-(p.z-.75) <= 2.2
                              and abs(x-p.x) <= 12 and abs(y-p.y) <= 12]
                    pose = (p.x, p.y, yaw)
                    wall_clear = swept_rectangle_clear(
                        points, v, w, self.motion_horizon,
                        self.half_length, self.half_width, self.margin)
                    if not wall_clear:
                        reduced_w = self.safe_reduced_turn(points, pose, v, w)
                        if reduced_w is None:
                            reason = 'wall_swept_footprint'
                        else:
                            output.twist.angular.z = reduced_w
                            reason = 'turn_speed_limited'
                    elif not swept_fire_clear(list(self.zones.values()), pose, v,w,self.motion_horizon,
                                              math.hypot(self.half_length,self.half_width)+self.margin):
                        reason = 'fire_boundary'
                    else:
                        output.twist.linear.x, output.twist.angular.z = v,w
            self.output.publish(output)
            if reason != getattr(self, 'last_reason', None):
                self.reason.publish(String(data=reason))
                self.navigation_reason.publish(String(data=reason))
                rospy.loginfo('Navigation safety: %s', reason)
                self.last_reason = reason
            if (now-self.last_diagnostic).to_sec() >= 1.0:
                self.last_diagnostic = now
                v = self.raw.twist.linear.x if self.raw is not None else 0.0
                w = self.raw.twist.angular.z if self.raw is not None else 0.0
                diagnostic = ('reason=%s odom_age=%.3f scan_age=%.3f points=%d '
                              'requested=(%.3f,%.3f) ros_output=(%.3f,%.3f)' % (
                    reason, odom_age, scan_age, len(self.scan[1]) if self.scan else 0,
                    v,w,output.twist.linear.x,output.twist.angular.z))
                diagnostic += ' ' + self.execution_diagnostic(now)
                physics, received = self.physics_diagnostic
                age = (now-received).to_sec() if received is not None else float('inf')
                diagnostic += ' physics_age=%.3f physics=[%s]' % (age, physics)
                self.diagnostic_pub.publish(String(data=diagnostic))
                rospy.loginfo(diagnostic)

    def execution_diagnostic(self, now):
        """Keep commanded Unity motion distinct from measured odometry."""
        command = self.unity_command
        unity_age = (now-command.header.stamp).to_sec() if command is not None else float('inf')
        unity = ('(%.4f,%.4f)' % (command.twist.linear.x, command.twist.angular.z)
                 if command is not None else 'unavailable')
        status, received = self.unity_status
        status_age = (now-received).to_sec() if received is not None else float('inf')
        if self.odom is None:
            measured = position = 'unavailable'
        else:
            t, p = self.odom.twist.twist, self.odom.pose.pose.position
            measured = '(%.4f,%.4f,%.4f)' % (t.linear.x, t.linear.y, t.angular.z)
            position = '(%.4f,%.4f)' % (p.x, p.y)
        return ('unity_command=%s unity_age=%.3f unity_status=%s status_age=%.3f '
                'odom_vx_vy_w=%s position=%s' % (
                    unity, unity_age, status, status_age, measured, position))


if __name__ == '__main__':
    rospy.init_node('navigation_safety_guard')
    NavigationSafetyGuard()
    rospy.spin()
