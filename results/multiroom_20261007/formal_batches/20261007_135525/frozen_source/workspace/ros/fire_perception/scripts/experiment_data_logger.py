#!/usr/bin/env python3
import csv
import hashlib
import json
import math
import os
import threading
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import Bool, Int8, Int32, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class ExperimentDataLogger:
    SUMMARY_FIELDS = [
        "trial_id", "map_name", "start_time", "end_time", "duration_s",
        "path_length_m", "total_victims", "rescued_victims",
        "remaining_victims", "mission_success", "recovery_attempts",
        "collision_count", "min_fire_distance_m", "final_status",
        "stationary_time_s", "recovery_time_s", "recovery_transitions",
    ]

    def __init__(self):
        self.output_directory = os.path.expanduser(rospy.get_param(
            "~experiment_output_directory", "/root/fire_nav_experiments"
        ))
        self.default_map_name = str(rospy.get_param(
            "~experiment_map_name", "complex"
        ))
        self.sample_interval = max(0.05, float(rospy.get_param(
            "~experiment_sample_interval", 0.2
        )))
        self.max_position_jump = max(1.0, float(rospy.get_param(
            "~experiment_max_position_jump", 5.0
        )))
        self.auto_record = bool(rospy.get_param(
            "~experiment_auto_record", True
        ))
        self.expected_victim_count = max(1, int(rospy.get_param(
            "~mission_total_victims", 5
        )))
        self.max_duration = max(0.0, float(rospy.get_param(
            "~experiment_max_duration_seconds", 0.0
        )))

        self.lock = threading.RLock()
        self.require_scene_counts = bool(rospy.get_param("~require_scene_counts", False))
        self.count_episode = None
        self.recording = False
        self.auto_armed = True
        self.trial_id = ""
        self.map_name = self.default_map_name
        self.started_at = None
        self.last_position = None
        self.last_sample_time = rospy.Time(0)
        self.path_length = 0.0
        self.rescued_victims = 0
        self.remaining_victims = -1
        self.total_victims = -1
        self.collision_count = 0
        self.collision_count_at_start = 0
        self.min_fire_distance = float("inf")
        self.mission_status = "unknown"
        self.previous_status = ""
        self.recovery_attempts = 0
        self.recovery_transitions = 0
        self.recovery_state = "unknown"
        self.mission_state = "unknown"
        self.target = None
        self.victim_detected = False
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.unity_guard_status = "unavailable"
        self.unity_applied_linear = None
        self.unity_applied_angular = None
        self.unity_feedback_at = rospy.Time(0)
        self.stationary_time = 0.0
        self.recovery_time = 0.0
        self.last_motion_sample_at = None
        self.event_file = None
        self.event_writer = None
        self.trial_directory = None
        self.fire_markers = {}
        self.trajectory_file = None
        self.trajectory_writer = None
        self.completion_timer = None

        os.makedirs(self.output_directory, exist_ok=True)
        rospy.Subscriber("/victim_call/scene_state", String,
                         self.scene_counts_callback, queue_size=5)
        rospy.Subscriber("/recovery_test/config", String, self.test_config_callback, queue_size=1)
        self.scene_layout = None
        rospy.Subscriber("/experiment/layout", String, self.layout_callback, queue_size=1)
        self.status_publisher = rospy.Publisher(
            "/experiment_logger/status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/state_estimation", Odometry, self.odom_callback, queue_size=10
        )
        rospy.Subscriber(
            "/victim_mission/rescued_count", Int32,
            self.rescued_callback, queue_size=5
        )
        rospy.Subscriber(
            "/victim_call/remaining", Int32,
            self.remaining_callback, queue_size=5
        )
        rospy.Subscriber(
            "/victim_call/total", Int32,
            self.total_callback, queue_size=5
        )
        rospy.Subscriber(
            "/victim_mission/status", String,
            self.mission_status_callback, queue_size=5
        )
        rospy.Subscriber("/victim_mission/state", String,
                         self.mission_state_callback, queue_size=5)
        rospy.Subscriber("/local_recovery/state", String,
                         self.recovery_state_callback, queue_size=5)
        rospy.Subscriber("/victim_mission/target", PointStamped,
                         self.target_callback, queue_size=5)
        rospy.Subscriber("/victim_call/detected", Bool,
                         self.victim_detected_callback, queue_size=5)
        self.visual_detected = False
        rospy.Subscriber("/victim_detection/detected", Bool,
                         self.visual_detected_callback, queue_size=5)
        rospy.Subscriber("/cmd_vel", TwistStamped,
                         self.cmd_vel_callback, queue_size=10)
        self.pipeline_log_times = {}
        rospy.Subscriber("/victim_mission/route_validation", String,
                         lambda msg: self.pipeline_event("ROUTE_VALIDATION", msg.data), queue_size=5)
        rospy.Subscriber("/far_planning_result", String,
                         lambda msg: self.pipeline_event("FAR_RESULT", msg.data), queue_size=1)
        rospy.Subscriber("/cmd_vel_raw", TwistStamped, self.raw_diagnostic_callback, queue_size=1)
        rospy.Subscriber("/path", Path, self.path_diagnostic_callback, queue_size=1)
        rospy.Subscriber("/way_point", PointStamped, self.waypoint_diagnostic_callback, queue_size=1)
        rospy.Subscriber("/stop", Int8, self.stop_diagnostic_callback, queue_size=1)
        rospy.Subscriber("/unity_motion_guard/status", String,
                         self.unity_guard_callback, queue_size=5)
        rospy.Subscriber("/local_recovery/guard_reason", String,
                         self.local_guard_callback, queue_size=5)
        rospy.Subscriber("/unity_motion_guard/applied_cmd", TwistStamped,
                         self.unity_applied_callback, queue_size=5)
        rospy.Subscriber(
            "/collision_count", UInt32,
            self.collision_callback, queue_size=5
        )
        rospy.Subscriber(
            "/fire_detection/keepout_markers", MarkerArray,
            self.fire_markers_callback, queue_size=2
        )

        rospy.Service("/experiment_logger/start", Trigger, self.start_service)
        rospy.Service("/experiment_logger/stop", Trigger, self.stop_service)
        rospy.Service("/experiment_logger/status", Trigger, self.status_service)
        self.timeout_timer = rospy.Timer(
            rospy.Duration(1.0), self.timeout_callback
        )
        rospy.on_shutdown(self.shutdown)
        self.publish_status("idle: ready to record")
        rospy.loginfo("Experiment logger ready: output=%s", self.output_directory)

    def current_map_name(self):
        if getattr(self, "last_test_config", None):
            return self.last_test_config
        if self.scene_layout and self.scene_layout.get("version"):
            return str(self.scene_layout["version"])
        return str(rospy.get_param(
            "/experiment_map_name", self.default_map_name
        ))

    def layout_callback(self, message):
        try:
            layout = json.loads(message.data)
            if isinstance(layout, dict) and "seed" in layout and "objects" in layout:
                with self.lock:
                    self.scene_layout = layout
                    if not getattr(self, "last_test_config", None) and layout.get("version"):
                        self.map_name = str(layout["version"])
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5.0, "Invalid experiment layout JSON")

    def start_service(self, _request):
        with self.lock:
            if self.recording:
                return TriggerResponse(False, "Already recording: {}".format(
                    self.trial_id
                ))
            now = datetime.now()
            self.trial_id = now.strftime("%Y%m%d_%H%M%S")
            self.map_name = self.current_map_name()
            self.started_at = now
            self.last_position = None
            self.last_sample_time = rospy.Time(0)
            self.path_length = 0.0
            self.recovery_attempts = 0
            self.recovery_transitions = 0
            self.stationary_time = 0.0
            self.recovery_time = 0.0
            self.last_motion_sample_at = None
            self.previous_status = self.mission_status
            self.min_fire_distance = float("inf")
            self.collision_count_at_start = self.collision_count
            self.open_trajectory_file()
            self.open_event_file()
            self.recording = True
            self.write_event("TRIAL_START", "map={}".format(self.map_name))
            self.write_event("VISUAL_INITIAL_STATE", int(self.visual_detected))
            self.publish_status("recording: {} map={}".format(
                self.trial_id, self.map_name
            ))
            rospy.logwarn(
                "Experiment recording started: trial=%s map=%s",
                self.trial_id, self.map_name
            )
            return TriggerResponse(True, "Recording started: {}".format(
                self.trial_id
            ))

    def stop_service(self, _request):
        with self.lock:
            if not self.recording:
                return TriggerResponse(False, "No experiment is recording")
            path = self.finalize("manual_stop")
            self.auto_armed = False
            return TriggerResponse(True, "Experiment saved to {}".format(path))

    def status_service(self, _request):
        with self.lock:
            if not self.recording:
                return TriggerResponse(True, "idle; output={}".format(
                    self.output_directory
                ))
            return TriggerResponse(True, (
                "recording {}; path={:.2f}m; rescued={}; remaining={}; "
                "recoveries={}".format(
                    self.trial_id, self.path_length, self.rescued_victims,
                    self.remaining_victims, self.recovery_attempts
                )
            ))

    def open_trajectory_file(self):
        self.trial_directory = os.path.join(self.output_directory, self.trial_id)
        os.makedirs(self.trial_directory, exist_ok=True)
        path = os.path.join(
            self.trial_directory,
            "trajectory_{}.csv".format(self.trial_id)
        )
        self.trajectory_file = open(path, "w", newline="")
        self.trajectory_writer = csv.DictWriter(
            self.trajectory_file,
            fieldnames=[
                "elapsed_s", "x", "y", "z", "path_length_m",
                "rescued_victims", "remaining_victims",
                "fire_distance_m", "mission_status",
                "mission_state", "recovery_state", "victim_detected",
                "target_x", "target_y", "target_distance_m",
                "odom_speed_mps", "cmd_linear_mps", "cmd_angular_rps",
                "unity_guard_status", "unity_applied_linear_mps",
                "unity_applied_angular_rps", "unity_feedback_age_s",
            ],
        )
        self.trajectory_writer.writeheader()
        self.trajectory_file.flush()

    def open_event_file(self):
        path = os.path.join(self.trial_directory or self.output_directory,
                            "events_{}.csv".format(self.trial_id))
        self.event_file = open(path, "w", newline="")
        self.event_writer = csv.DictWriter(
            self.event_file,
            fieldnames=["elapsed_s", "event", "value", "mission_state",
                        "recovery_state", "rescued_victims"])
        self.event_writer.writeheader()
        self.event_file.flush()

    def write_event(self, event, value=""):
        if self.event_writer is None or self.started_at is None:
            return
        elapsed = (datetime.now() - self.started_at).total_seconds()
        self.event_writer.writerow({
            "elapsed_s": "{:.3f}".format(elapsed), "event": event,
            "value": value, "mission_state": self.mission_state,
            "recovery_state": self.recovery_state,
            "rescued_victims": self.rescued_victims,
        })
        self.event_file.flush()

    def odom_callback(self, message):
        with self.lock:
            if not self.recording:
                return
            now = rospy.Time.now()
            point = message.pose.pose.position
            position = (point.x, point.y, point.z)
            velocity = message.twist.twist.linear
            odom_speed = math.hypot(velocity.x, velocity.y)
            if self.last_position is not None:
                step = math.hypot(
                    position[0] - self.last_position[0],
                    position[1] - self.last_position[1]
                )
                if step <= self.max_position_jump:
                    self.path_length += step
                else:
                    rospy.logwarn("Experiment logger ignored %.2f m pose jump", step)
            self.last_position = position

            fire_distance = self.distance_to_nearest_fire(position)
            if fire_distance is not None:
                self.min_fire_distance = min(
                    self.min_fire_distance, fire_distance
                )
            if (now - self.last_sample_time).to_sec() < self.sample_interval:
                return
            self.last_sample_time = now
            elapsed = (datetime.now() - self.started_at).total_seconds()
            if self.last_motion_sample_at is not None:
                dt = max(0.0, (now - self.last_motion_sample_at).to_sec())
                if odom_speed < 0.03 and abs(self.command_linear) > 0.05:
                    self.stationary_time += dt
                if self.recovery_state not in ("normal", "unknown", ""):
                    self.recovery_time += dt
            self.last_motion_sample_at = now
            target_distance = None
            if self.target is not None:
                target_distance = math.hypot(position[0] - self.target[0],
                                             position[1] - self.target[1])
            self.trajectory_writer.writerow({
                "elapsed_s": "{:.3f}".format(elapsed),
                "x": "{:.4f}".format(position[0]),
                "y": "{:.4f}".format(position[1]),
                "z": "{:.4f}".format(position[2]),
                "path_length_m": "{:.3f}".format(self.path_length),
                "rescued_victims": self.rescued_victims,
                "remaining_victims": self.remaining_victims,
                "fire_distance_m": (
                    "{:.3f}".format(fire_distance)
                    if fire_distance is not None else ""
                ),
                "mission_status": self.mission_status,
                "mission_state": self.mission_state,
                "recovery_state": self.recovery_state,
                "victim_detected": int(self.victim_detected),
                "target_x": "{:.4f}".format(self.target[0]) if self.target else "",
                "target_y": "{:.4f}".format(self.target[1]) if self.target else "",
                "target_distance_m": "{:.3f}".format(target_distance) if target_distance is not None else "",
                "odom_speed_mps": "{:.3f}".format(odom_speed),
                "cmd_linear_mps": "{:.3f}".format(self.command_linear),
                "cmd_angular_rps": "{:.3f}".format(self.command_angular),
                "unity_guard_status": self.unity_guard_status,
                "unity_applied_linear_mps": self.unity_applied_linear,
                "unity_applied_angular_rps": self.unity_applied_angular,
                "unity_feedback_age_s": (
                    (now - self.unity_feedback_at).to_sec()
                    if self.unity_applied_linear is not None else ""),
            })
            self.trajectory_file.flush()

    def rescued_callback(self, message):
        with self.lock:
            if self.require_scene_counts:
                return
            previous = self.rescued_victims
            self.rescued_victims = min(
                self.expected_victim_count, max(0, int(message.data))
            )
            self.update_total_victims()
            if self.recording and self.rescued_victims != previous:
                self.write_event("RESCUED_COUNT", self.rescued_victims)

    def mission_state_callback(self, message):
        with self.lock:
            state = message.data.strip()
            changed = state != self.mission_state
            self.mission_state = state
            if self.recording and changed:
                self.write_event("MISSION_STATE", state)

    def recovery_state_callback(self, message):
        with self.lock:
            state = message.data.strip()
            changed = state != self.recovery_state
            self.recovery_state = state
            if self.recording and changed:
                self.recovery_transitions += 1
                self.write_event("RECOVERY_STATE", state)

    def target_callback(self, message):
        with self.lock:
            new_target = (message.point.x, message.point.y)
            if self.recording and (self.target is None or
                    math.hypot(new_target[0] - self.target[0],
                               new_target[1] - self.target[1]) > 0.25):
                self.write_event("TARGET", "{:.3f},{:.3f}".format(*new_target))
            self.target = new_target

    def victim_detected_callback(self, message):
        with self.lock:
            detected = bool(message.data)
            if self.recording and detected != self.victim_detected:
                self.write_event("VICTIM_DETECTED", int(detected))
            self.victim_detected = detected

    def visual_detected_callback(self, message):
        # Keep the historical call event unchanged; add an unambiguous visual event.
        with self.lock:
            detected = bool(message.data)
            if self.recording and detected != self.visual_detected:
                self.write_event("VISUAL_DETECTED", int(detected))
            self.visual_detected = detected

    def cmd_vel_callback(self, message):
        with self.lock:
            self.command_linear = message.twist.linear.x
            self.command_angular = message.twist.angular.z

    def unity_guard_callback(self, message):
        with self.lock:
            if message.data != self.unity_guard_status and self.recording:
                self.write_event("UNITY_MOTION_GUARD", message.data)
            self.unity_guard_status = message.data

    def local_guard_callback(self, message):
        with self.lock:
            if self.recording:
                self.write_event("LOCAL_SAFETY_GUARD", message.data)

    def unity_applied_callback(self, message):
        with self.lock:
            self.unity_applied_linear = message.twist.linear.x
            self.unity_applied_angular = message.twist.angular.z
            self.unity_feedback_at = rospy.Time.now()

    def pipeline_event(self, name, value):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if now - self.pipeline_log_times.get(name, -float("inf")) < 1.0:
                return
            self.pipeline_log_times[name] = now
            self.write_event(name, value)

    def raw_diagnostic_callback(self, msg):
        self.pipeline_event("RAW_COMMAND", "stamp=%.3f v=%.3f w=%.3f" % (
            msg.header.stamp.to_sec(), msg.twist.linear.x, msg.twist.angular.z))

    def path_diagnostic_callback(self, msg):
        end = msg.poses[-1].pose.position if msg.poses else None
        self.pipeline_event("LOCAL_PATH", "stamp=%.3f frame=%s count=%d endpoint=%s" % (
            msg.header.stamp.to_sec(), msg.header.frame_id, len(msg.poses),
            "%.3f,%.3f" % (end.x,end.y) if end else "none"))

    def waypoint_diagnostic_callback(self, msg):
        self.pipeline_event("WAYPOINT", "stamp=%.3f frame=%s x=%.3f y=%.3f" % (
            msg.header.stamp.to_sec(), msg.header.frame_id, msg.point.x,msg.point.y))

    def stop_diagnostic_callback(self, msg):
        with self.lock:
            if getattr(self, "last_diagnostic_stop", None) != msg.data:
                self.last_diagnostic_stop = msg.data
                self.write_event("STOP_COMMAND", str(msg.data))

    def remaining_callback(self, message):
        with self.lock:
            if self.require_scene_counts:
                return
            self.remaining_victims = max(0, int(message.data))
            self.update_total_victims()
            self.maybe_auto_start()

    def total_callback(self, message):
        with self.lock:
            if self.require_scene_counts:
                return
            if message.data <= 0:
                return
            self.expected_victim_count = int(message.data)
            self.update_total_victims()

    def scene_counts_callback(self, message):
        if not self.require_scene_counts:
            return
        try:
            episode, total_text, rescued_text, _ = message.data.split("|")
            total = int(total_text)
            rescued = set(filter(None, rescued_text.split(",")))
            if not episode or total <= 0 or len(rescued) > total:
                return
        except (ValueError, AttributeError):
            return
        with self.lock:
            if self.count_episode is not None and episode != self.count_episode:
                rospy.logerr_throttle(5.0, "Scene episode changed; restart pilot logger")
                return
            self.count_episode = episode
            changed = (self.total_victims, self.rescued_victims) != (total, len(rescued))
            self.expected_victim_count = self.total_victims = total
            self.rescued_victims = len(rescued)
            self.remaining_victims = total - len(rescued)
            self.maybe_auto_start()
            if changed and self.recording:
                self.write_event("SCENE_COUNTS", message.data)

    def update_total_victims(self):
        self.total_victims = self.expected_victim_count
        if self.remaining_victims >= 0:
            self.remaining_victims = min(
                self.expected_victim_count, self.remaining_victims
            )
            self.rescued_victims = (
                self.expected_victim_count - self.remaining_victims
            )

    def timeout_callback(self, _event):
        with self.lock:
            if (
                not self.recording
                or self.max_duration <= 0.0
                or self.started_at is None
            ):
                return
            elapsed = (datetime.now() - self.started_at).total_seconds()
            if elapsed >= self.max_duration:
                rospy.logwarn(
                    "Experiment timed out after %.1f seconds", elapsed
                )
                self.mission_status = "experiment timeout"
                self.finalize("timeout")
                self.auto_armed = False

    def collision_callback(self, message):
        with self.lock:
            self.collision_count = int(message.data)

    def test_config_callback(self, message):
        with self.lock:
            self.map_name = message.data
            self.expected_victim_count = 0
            self.total_victims = 0
            self.rescued_victims = 0
            self.remaining_victims = 0
            if self.recording:
                if getattr(self, "last_test_config", None) != message.data:
                    self.write_event("TEST_CONFIG", message.data)
            self.last_test_config = message.data
            self.maybe_auto_start()

    def mission_status_callback(self, message):
        with self.lock:
            status = message.data.strip()
            if status == "navigation test reached goal":
                self.mission_status = status
                self.auto_armed = False
                self.finalize("navigation_test_complete")
                return
            previous_status = self.previous_status
            if status == "mission reset" or status == "mission started":
                self.auto_armed = True
            elif status == "mission stopped":
                self.auto_armed = False
            if (self.recording and status.startswith("exploring ")
                    and status != self.previous_status):
                self.recovery_attempts += 1
            self.mission_status = status
            self.previous_status = status
            if self.recording and status != previous_status:
                self.write_event("MISSION_STATUS", status)
            if status.startswith(("recovery failed:", "mission timeout")):
                self.auto_armed = False
                if self.recording:
                    self.finalize("recovery_failed" if status.startswith("recovery failed:")
                                  else "mission_timeout")
                return
            if self.recording and status.startswith("mission complete"):
                # Vict mission manager publishes status before its final
                # rescued_count update. Give the queued count/remaining
                # callbacks one short ROS cycle before sealing the files.
                if self.completion_timer is None:
                    self.completion_timer = rospy.Timer(
                        rospy.Duration(0.35),
                        self.finalize_complete_callback,
                        oneshot=True,
                    )
                self.auto_armed = False
            else:
                self.maybe_auto_start()

    def finalize_complete_callback(self, _event):
        with self.lock:
            self.completion_timer = None
            if self.recording:
                self.finalize("mission_complete")

    def maybe_auto_start(self):
        if (
            not self.auto_record
            or not self.auto_armed
            or self.recording
            or (self.remaining_victims <= 0 and not getattr(self, "last_test_config", None))
        ):
            return
        inactive_statuses = (
            "unknown",
            "mission complete",
            "mission stopped",
            "mission reset",
            "mission timeout",
            "recovery failed:",
            "navigation test reached goal",
        )
        if self.mission_status.startswith(inactive_statuses):
            return
        response = self.start_service(None)
        if response.success:
            rospy.loginfo("Experiment auto-recording triggered by mission data")

    def fire_markers_callback(self, message):
        with self.lock:
            for marker in message.markers:
                key = (marker.ns, marker.id)
                if marker.action == Marker.DELETEALL:
                    self.fire_markers.clear()
                elif marker.action == Marker.DELETE:
                    self.fire_markers.pop(key, None)
                elif marker.type in (Marker.SPHERE, Marker.CYLINDER):
                    self.fire_markers[key] = (
                        marker.pose.position.x, marker.pose.position.y
                    )

    def distance_to_nearest_fire(self, position):
        if not self.fire_markers:
            return None
        return min(
            math.hypot(position[0] - fire[0], position[1] - fire[1])
            for fire in self.fire_markers.values()
        )

    def finalize(self, reason):
        if not self.recording:
            return ""
        ended_at = datetime.now()
        duration = (ended_at - self.started_at).total_seconds()
        success = (self.mission_status.startswith("mission complete")
                   or self.mission_status == "navigation test reached goal")
        collision_delta = max(
            0, self.collision_count - self.collision_count_at_start
        )
        # Always append a terminal trajectory record after the latest mission
        # callbacks. This keeps the detailed CSV consistent with trials.csv,
        # even when completion occurs between periodic odometry samples.
        if self.trajectory_writer is not None and self.last_position is not None:
            fire_distance = self.distance_to_nearest_fire(self.last_position)
            self.trajectory_writer.writerow({
                "elapsed_s": "{:.3f}".format(duration),
                "x": "{:.4f}".format(self.last_position[0]),
                "y": "{:.4f}".format(self.last_position[1]),
                "z": "{:.4f}".format(self.last_position[2]),
                "path_length_m": "{:.3f}".format(self.path_length),
                "rescued_victims": self.rescued_victims,
                "remaining_victims": self.remaining_victims,
                "fire_distance_m": (
                    "{:.3f}".format(fire_distance)
                    if fire_distance is not None else ""
                ),
                "mission_status": "{} ({})".format(
                    self.mission_status, reason
                ),
                "mission_state": self.mission_state,
                "recovery_state": self.recovery_state,
                "victim_detected": int(self.victim_detected),
                "target_x": "{:.4f}".format(self.target[0]) if self.target else "",
                "target_y": "{:.4f}".format(self.target[1]) if self.target else "",
                "target_distance_m": "",
                "odom_speed_mps": "",
                "cmd_linear_mps": "{:.3f}".format(self.command_linear),
                "cmd_angular_rps": "{:.3f}".format(self.command_angular),
            })
            self.trajectory_file.flush()
        # Failed/interrupted runs are deliberately discarded.  A complete run
        # is self-contained in its own directory and can be replayed later.
        if not success and not rospy.get_param("~experiment_keep_failed", False):
            for handle in (self.trajectory_file, self.event_file):
                if handle is not None:
                    handle.flush()
                    handle.close()
            if self.trial_directory and os.path.isdir(self.trial_directory):
                for name in os.listdir(self.trial_directory):
                    try:
                        os.remove(os.path.join(self.trial_directory, name))
                    except OSError:
                        pass
                try:
                    os.rmdir(self.trial_directory)
                except OSError:
                    pass
            completed_trial = self.trial_id
            self.trajectory_file = None
            self.trajectory_writer = None
            self.event_file = None
            self.event_writer = None
            self.trial_directory = None
            self.recording = False
            self.publish_status("discarded: {} success=False".format(completed_trial))
            rospy.logwarn("Experiment discarded: trial=%s reason=%s", completed_trial, reason)
            return ""

        summary_path = os.path.join(self.output_directory, "trials.csv")
        # Do not append a wider diagnostic row beneath a legacy header. Keep
        # historical results intact and start a clearly named v2 summary.
        if os.path.exists(summary_path) and os.path.getsize(summary_path) > 0:
            with open(summary_path, "r", newline="") as existing_file:
                existing_header = next(csv.reader(existing_file), [])
            if existing_header != self.SUMMARY_FIELDS:
                summary_path = os.path.join(
                    self.output_directory, "trials_diagnostic_v2.csv"
                )
        write_header = (
            not os.path.exists(summary_path)
            or os.path.getsize(summary_path) == 0
        )
        with open(summary_path, "a", newline="") as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=self.SUMMARY_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow({
                "trial_id": self.trial_id,
                "map_name": self.map_name,
                "start_time": self.started_at.isoformat(timespec="seconds"),
                "end_time": ended_at.isoformat(timespec="seconds"),
                "duration_s": "{:.3f}".format(duration),
                "path_length_m": "{:.3f}".format(self.path_length),
                "total_victims": self.total_victims,
                "rescued_victims": self.rescued_victims,
                "remaining_victims": self.remaining_victims,
                "mission_success": success,
                "recovery_attempts": self.recovery_attempts,
                "collision_count": collision_delta,
                "min_fire_distance_m": (
                    "{:.3f}".format(self.min_fire_distance)
                    if math.isfinite(self.min_fire_distance) else ""
                ),
                "final_status": "{} ({})".format(self.mission_status, reason),
                "stationary_time_s": "{:.3f}".format(self.stationary_time),
                "recovery_time_s": "{:.3f}".format(self.recovery_time),
                "recovery_transitions": self.recovery_transitions,
            })
        if self.trajectory_file is not None:
            self.trajectory_file.flush()
            self.trajectory_file.close()
        if self.event_file is not None:
            self.write_event("TRIAL_END", reason)
            self.event_file.flush()
            self.event_file.close()
        self.write_reproducibility_bundle(ended_at, reason)
        completed_trial = self.trial_id
        self.trajectory_file = None
        self.trajectory_writer = None
        self.event_file = None
        self.trial_directory = None
        self.event_writer = None
        self.recording = False
        self.publish_status("saved: {} success={}".format(
            completed_trial, success
        ))
        rospy.logwarn(
            "Experiment saved: trial=%s success=%s duration=%.1fs path=%.2fm",
            completed_trial, success, duration, self.path_length
        )
        return summary_path

    def write_reproducibility_bundle(self, ended_at, reason):
        """Write metadata and SHA256 manifest next to every successful run."""
        if not self.trial_directory:
            return
        source_hashes = {}
        source_candidates = [
            os.path.abspath(__file__),
            os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "victim_color_detector.yaml"),
            os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch", "fire_color_detector.launch"),
        ]
        for source in source_candidates:
            if not os.path.isfile(source):
                continue
            digest = hashlib.sha256()
            with open(source, "rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            source_hashes[source] = digest.hexdigest()
        metadata = {
            "scene_layout": self.scene_layout,
            "trial_id": self.trial_id,
            "map_name": self.map_name,
            "start_time": self.started_at.isoformat(timespec="seconds"),
            "end_time": ended_at.isoformat(timespec="seconds"),
            "duration_s": (ended_at - self.started_at).total_seconds(),
            "path_length_m": self.path_length,
            "total_victims": self.total_victims,
            "rescued_victims": self.rescued_victims,
            "reason": reason,
            "mission_status": self.mission_status,
            "recovery_state": self.recovery_state,
            "source_sha256": source_hashes,
            "ros_params": {
                "experiment_sample_interval": self.sample_interval,
                "experiment_max_position_jump": self.max_position_jump,
                "mission_total_victims": self.expected_victim_count,
            },
            "replay": {
                "launch": "roslaunch fire_perception fire_color_detector.launch",
                "notes": "Start Unity with the recorded scene/map, then replay the same launch and compare trajectory/events.",
            },
        }
        metadata_path = os.path.join(self.trial_directory, "metadata.json")
        with open(metadata_path, "w") as stream:
            json.dump(metadata, stream, indent=2, sort_keys=True)
        manifest_path = os.path.join(self.trial_directory, "SHA256SUMS.txt")
        files = []
        for root, _, names in os.walk(self.trial_directory):
            for name in sorted(names):
                if name == "SHA256SUMS.txt":
                    continue
                path = os.path.join(root, name)
                digest = hashlib.sha256()
                with open(path, "rb") as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(chunk)
                files.append("{}  {}".format(
                    digest.hexdigest(), os.path.relpath(path, self.trial_directory)))
        with open(manifest_path, "w") as stream:
            stream.write("\n".join(files) + "\n")

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def shutdown(self):
        with self.lock:
            if self.recording:
                self.finalize("node_shutdown")


def main():
    rospy.init_node("experiment_data_logger")
    ExperimentDataLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
