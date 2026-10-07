#!/usr/bin/env python3
import math
import threading
import os
import sys
from collections import deque
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from observed_route import observed_route

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Pose, PoseArray, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Int8, Int32, String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class VictimTrack:
    def __init__(self, track_id, pose, now):
        self.track_id = track_id
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        self.confirmations = 1
        self.last_seen = now
        self.rescued = False
        self.deferred_until = rospy.Time(0)


class VictimMissionManager:
    def __init__(self):
        # Read once per process. Paired trials restart the manager, never toggle mid-run.
        self.enable_simulated_call_guidance = bool(
            rospy.get_param("~enable_simulated_call_guidance", True))
        self.world_frame = rospy.get_param("~output_frame", "map")
        self.confirmation_count = max(
            1, int(rospy.get_param("~victim_confirmation_count", 3))
        )
        self.merge_distance = float(
            rospy.get_param("~victim_track_merge_distance", 1.0)
        )
        self.update_alpha = float(
            rospy.get_param("~victim_track_update_alpha", 0.2)
        )
        self.unconfirmed_timeout = float(
            rospy.get_param("~unconfirmed_track_timeout", 2.0)
        )
        self.rescue_distance = float(rospy.get_param("~rescue_distance", 1.5))
        self.navigation_standoff_distance = min(
            max(
                0.0,
                float(rospy.get_param(
                    "~victim_navigation_standoff_distance", 2.7
                )),
            ),
            max(0.0, self.rescue_distance - 0.15),
        )
        self.standoff_retry_timeout = float(rospy.get_param(
            "~victim_standoff_retry_timeout", 12.0
        ))
        self.standoff_progress_distance = float(rospy.get_param(
            "~victim_standoff_progress_distance", 0.5
        ))
        self.standoff_angle_offsets = [
            math.radians(float(value))
            for value in rospy.get_param(
                "~victim_standoff_angle_offsets_degrees",
                [0.0, 60.0, -60.0, 120.0, -120.0, 180.0],
            )
        ]
        self.standoff_obstacle_clearance = float(rospy.get_param(
            "~victim_standoff_obstacle_clearance", 2.2
        ))
        self.standoff_zero_command_timeout = float(rospy.get_param(
            "~victim_standoff_zero_command_timeout", 3.0
        ))
        self.standoff_defer_seconds = float(rospy.get_param(
            "~victim_standoff_defer_seconds", 45.0
        ))
        self.rescued_suppression_distance = float(
            rospy.get_param(
                "~rescued_suppression_distance",
                max(self.rescue_distance, self.merge_distance),
            )
        )
        self.rescue_hold = float(
            rospy.get_param("~rescue_hold_seconds", 2.0)
        )
        self.recovery_inhibit_distance = max(
            self.rescue_distance,
            float(rospy.get_param(
                "~victim_recovery_inhibit_distance",
                self.rescue_distance + 0.6,
            )),
        )
        self.precise_target_range = float(
            rospy.get_param("~victim_precise_target_range", 12.0)
        )
        self.contact_rescue_distance = float(
            rospy.get_param("~contact_rescue_distance", 0.8)
        )
        self.contact_rescue_hold = float(
            rospy.get_param("~contact_rescue_hold_seconds", 1.0)
        )
        self.enable_legacy_distance_rescue = bool(
            rospy.get_param("~enable_legacy_distance_rescue", False)
        )
        self.republish_distance = float(
            rospy.get_param("~goal_republish_distance", 0.5)
        )
        self.goal_republish_interval = float(
            rospy.get_param("~goal_republish_interval", 2.0)
        )
        self.vehicle_height = float(
            rospy.get_param("~mission_vehicle_height", 0.75)
        )
        self.active = bool(rospy.get_param("~mission_auto_start", True))
        self.expected_victim_count = max(
            1, int(rospy.get_param("~mission_total_victims", 5))
        )
        self.mission_max_duration = max(
            0.0,
            float(rospy.get_param("~mission_max_duration_seconds", 1800.0)),
        )
        self.mission_started_at = None
        self.state = "LISTENING" if self.active else "IDLE"

        self.lock = threading.Lock()
        self.tracks = []
        self.next_track_id = 1
        self.rescued_positions = [
            (float(item[0]), float(item[1]), float(item[2]))
            for item in rospy.get_param(
                "/victim_mission_persisted_rescued_positions", []
            )
            if isinstance(item, (list, tuple)) and len(item) >= 3
        ]
        self.robot_position = None
        self.current_target_id = None
        self.standoff_candidate_index = 0
        self.standoff_base_angle = None
        self.standoff_best_distance = None
        self.standoff_progress_time = rospy.Time.now()
        self.standoff_attempted_indices = set()
        self.standoff_zero_since = None
        self.raw_linear_speed = 0.0
        self.raw_angular_speed = 0.0
        self.raw_command_received_at = rospy.Time(0)
        self.obstacle_points = []
        self.recovery_goal_clearance = math.hypot(
            float(rospy.get_param("/localPlanner/vehicleLength", 1.35)) / 2.0,
            float(rospy.get_param("/localPlanner/vehicleWidth", 1.80)) / 2.0,
        ) + 0.30 + 1.5
        self.last_goal_position = None
        self.last_goal_time = rospy.Time(0)
        self.arrival_started_at = None
        self.call_contact_started_at = None
        self.call_rescued_count = max(
            0,
            int(
                rospy.get_param(
                    "/victim_mission_persisted_rescued_count", 0
                )
            ),
        )
        self.contact_rescue_pending_remaining = None
        self.completion_candidate_since = None
        self.in_rescue_range = False
        self.rescue_zone_armed = True
        self.call_detected = False
        self.call_bearing = None
        self.call_strength = None
        self.call_precise_position = None
        self.call_precise_received_at = rospy.Time(0)
        self.call_received_at = rospy.Time(0)
        self.call_timeout = float(rospy.get_param("~victim_call_timeout", 1.0))
        self.visual_timeout = float(
            rospy.get_param("~victim_visual_timeout", 1.25)
        )
        self.visual_lost_grace = max(self.visual_timeout, float(
            rospy.get_param("~victim_visual_lost_grace_seconds", 3.0)
        ))
        self.visual_max_observation_age = float(rospy.get_param(
            "~victim_visual_max_observation_age", 1.5
        ))
        self.visual_direction_tolerance = math.radians(float(rospy.get_param(
            "~victim_visual_direction_tolerance_degrees", 30.0
        )))
        self.visual_goal_change_distance = float(rospy.get_param(
            "~victim_visual_goal_change_distance", 1.5
        ))
        self.visual_goal_arrival_distance = float(rospy.get_param(
            "~victim_visual_goal_arrival_distance", 1.5
        ))
        self.visual_goal_keepalive = float(rospy.get_param(
            "~victim_visual_goal_keepalive_seconds", 8.0
        ))
        self.visual_track_freshness = float(rospy.get_param(
            "~victim_visual_track_freshness_seconds", 3.0
        ))
        self.visual_confirmation_count = max(
            1, int(rospy.get_param("~victim_visual_confirmation_count", 3))
        )
        self.visual_goal_distance = float(
            rospy.get_param("~victim_visual_goal_distance", 6.0)
        )
        self.visual_goal_interval = float(
            rospy.get_param("~victim_visual_goal_interval", 1.0)
        )
        self.visual_acoustic_handoff_distance = float(
            rospy.get_param("~victim_visual_acoustic_handoff_distance", 3.5)
        )
        self.visual_rescued_suppression_timeout = float(
            rospy.get_param("~victim_visual_rescued_suppression_timeout", 3.0)
        )
        self.visual_heading_tolerance = math.radians(
            float(rospy.get_param("~victim_visual_heading_tolerance_degrees", 10.0))
        )
        self.call_goal_distance = float(
            rospy.get_param("~victim_call_goal_distance", 8.0)
        )
        self.call_goal_interval = float(
            rospy.get_param("~victim_call_goal_interval", 3.0)
        )
        self.call_goal_change_distance = float(
            rospy.get_param("~victim_call_goal_change_distance", 0.15)
        )
        self.acoustic_goal_update_distance = float(
            rospy.get_param("~victim_acoustic_goal_update_distance", 2.5)
        )
        self.acoustic_goal_update_interval = float(
            rospy.get_param("~victim_acoustic_goal_update_interval", 6.0)
        )
        self.call_hearing_distance = float(
            rospy.get_param("~victim_call_hearing_distance", 750.0)
        )
        self.call_standoff_distance = float(
            rospy.get_param("~victim_call_standoff_distance", 2.0)
        )
        self.call_min_goal_distance = float(
            rospy.get_param("~victim_call_min_goal_distance", 0.5)
        )
        self.call_turn_gain = float(
            rospy.get_param("~victim_call_turn_gain", 1.0)
        )
        self.call_max_turn_rate = float(
            rospy.get_param("~victim_call_max_turn_rate", 0.8)
        )
        # A nearby acoustic bearing can remain biased by wall reflections.
        # Never let the high-level manager hold the vehicle in an endless
        # stop-and-spin loop; after this window FAR/local planning gets a
        # short forward goal and can route around the occlusion.
        self.call_turn_hold_seconds = max(
            0.5,
            float(rospy.get_param("~victim_call_turn_hold_seconds", 3.0)),
        )
        self.call_turn_started_at = None
        self.call_heading_tolerance = math.radians(
            float(rospy.get_param("~victim_call_heading_tolerance_degrees", 8.0))
        )
        self.stuck_timeout = float(
            rospy.get_param("~wall_bypass_stuck_timeout", 5.0)
        )
        self.stuck_progress_distance = float(
            rospy.get_param("~wall_bypass_progress_distance", 0.4)
        )
        self.detour_distance = float(
            rospy.get_param("~wall_bypass_distance", 8.0)
        )
        self.detour_forward_bias = float(
            rospy.get_param("~wall_bypass_forward_bias", 1.5)
        )
        self.detour_duration = float(
            rospy.get_param("~wall_bypass_duration", 15.0)
        )
        self.detour_arrival_distance = float(
            rospy.get_param("~wall_bypass_arrival_distance", 1.5)
        )
        self.detour_republish_interval = float(
            rospy.get_param("~wall_bypass_republish_interval", 2.0)
        )
        self.detour_side_lock_duration = float(
            rospy.get_param("~wall_bypass_side_lock_duration", 90.0)
        )
        self.enable_wall_bypass = bool(
            rospy.get_param("~enable_wall_bypass", False)
        )
        self.use_topological_exploration = bool(
            rospy.get_param("~use_topological_exploration", True)
        )
        self.topology_topic = rospy.get_param(
            "~topology_topic", "/viz_graph_topic"
        )
        self.exploration_goal_arrival_distance = float(
            rospy.get_param("~exploration_goal_arrival_distance", 2.0)
        )
        self.exploration_visit_spacing = float(
            rospy.get_param("~exploration_visit_spacing", 2.5)
        )
        self.exploration_visited_radius = float(
            rospy.get_param("~exploration_visited_radius", 5.0)
        )
        self.exploration_node_min_distance = float(
            rospy.get_param("~exploration_node_min_distance", 4.0)
        )
        self.exploration_node_max_distance = float(
            rospy.get_param("~exploration_node_max_distance", 18.0)
        )
        self.exploration_min_direction_alignment = float(
            rospy.get_param("~exploration_min_direction_alignment", 0.15)
        )
        self.exploration_direction_weight = float(
            rospy.get_param("~exploration_direction_weight", 3.0)
        )
        self.exploration_novelty_weight = float(
            rospy.get_param("~exploration_novelty_weight", 0.2)
        )
        self.exploration_distance_weight = float(
            rospy.get_param("~exploration_distance_weight", 0.02)
        )
        self.exploration_history_limit = max(
            50, int(rospy.get_param("~exploration_history_limit", 600))
        )
        self.exploration_failed_goal_radius = float(
            rospy.get_param("~exploration_failed_goal_radius", 4.0)
        )
        self.exploration_failed_goal_limit = max(
            10, int(rospy.get_param("~exploration_failed_goal_limit", 100))
        )
        self.exploration_failed_goal_cooldown = float(
            rospy.get_param("~exploration_failed_goal_cooldown", 90.0)
        )
        self.fire_block_replan_delay = max(
            0.2, float(rospy.get_param("~fire_block_replan_delay", 1.2))
        )
        self.fire_block_replan_cooldown = max(
            1.0, float(rospy.get_param("~fire_block_replan_cooldown", 4.0))
        )
        self.wall_block_replan_delay = max(
            0.5, float(rospy.get_param("~wall_block_replan_delay", 2.0))
        )
        self.wall_block_replan_cooldown = max(
            1.0, float(rospy.get_param("~wall_block_replan_cooldown", 5.0))
        )
        self.exploration_failed_pocket_radius = max(
            0.5,
            float(rospy.get_param("~exploration_failed_pocket_radius", 3.5)),
        )
        self.exploration_failed_pocket_merge_distance = max(
            0.2,
            float(rospy.get_param("~exploration_failed_pocket_merge_distance", 1.5)),
        )
        self.exploration_failed_pocket_cooldown = max(
            1.0,
            float(rospy.get_param("~exploration_failed_pocket_cooldown", 1800.0)),
        )
        self.coverage_cell_size = max(
            1.0, float(rospy.get_param("~exploration_coverage_cell_size", 4.0))
        )
        self.coverage_observation_radius = max(
            self.coverage_cell_size,
            float(rospy.get_param("~exploration_coverage_observation_radius", 7.0)),
        )
        self.coverage_gain_radius = max(
            self.coverage_cell_size,
            float(rospy.get_param("~exploration_coverage_gain_radius", 12.0)),
        )
        self.coverage_gain_weight = float(
            rospy.get_param("~exploration_coverage_gain_weight", 4.0)
        )
        self.coverage_region_cooldown = float(
            rospy.get_param("~exploration_region_cooldown", 75.0)
        )
        self.coverage_recent_penalty = float(
            rospy.get_param("~exploration_recent_region_penalty", 3.0)
        )
        self.coverage_stall_timeout = float(
            rospy.get_param("~exploration_coverage_stall_timeout", 25.0)
        )
        self.loop_detection_window = float(
            rospy.get_param("~exploration_loop_detection_window", 20.0)
        )
        self.loop_min_duration = float(
            rospy.get_param("~exploration_loop_min_duration", 8.0)
        )
        self.loop_min_path_length = float(
            rospy.get_param("~exploration_loop_min_path_length", 8.0)
        )
        self.loop_closure_radius = float(
            rospy.get_param("~exploration_loop_closure_radius", 2.2)
        )
        self.loop_detection_cooldown = float(
            rospy.get_param("~exploration_loop_detection_cooldown", 30.0)
        )
        # Keep acoustic exploration goals inside the traversable map. These
        # limits deliberately sit several metres inside the physical walls so
        # FAR never receives a goal on, or just beyond, a boundary collider.
        self.map_min_x = float(rospy.get_param("~mission_map_min_x", -1.0e6))
        self.map_max_x = float(rospy.get_param("~mission_map_max_x", 1.0e6))
        self.map_min_y = float(rospy.get_param("~mission_map_min_y", -1.0e6))
        self.map_max_y = float(rospy.get_param("~mission_map_max_y", 1.0e6))
        self.last_call_goal_time = rospy.Time(0)
        self.last_call_goal_position = None
        self.scene_episode = None
        self.call_approach_goal_position = None
        self.call_approach_goal_time = rospy.Time(0)
        self.scene_rescued_ids = set()
        self.scene_call_id = None
        self.progress_anchor_position = None
        self.progress_anchor_time = rospy.Time.now()
        self.best_call_distance = None
        self.call_progress_time = rospy.Time.now()
        self.detour_goal = None
        self.detour_started_at = rospy.Time(0)
        self.detour_last_publish = rospy.Time(0)
        self.detour_best_remaining = None
        self.detour_progress_time = rospy.Time(0)
        self.detour_side = 1.0
        self.detour_locked_side = None
        self.detour_side_lock_until = rospy.Time(0)
        self.remaining_victims = None
        self.graph_nodes = []
        self.visited_positions = []
        self.failed_exploration_goals = []
        self.failed_exploration_pockets = []
        self.local_recovery_active = False
        self.replan_required = False
        self.replan_request_pending = False
        self.rejected_navigation_goal = None
        self.replan_fallback_side = 1.0
        self.replan_goal_min_distance = float(rospy.get_param(
            "~mission_replan_goal_min_distance", 1.2
        ))
        self.visual_bearing = None
        self.visual_world_angle = None
        self.visual_last_stamp = rospy.Time(0)
        self.visual_goal_position = None
        self.visual_guidance_active = False
        self.odom_yaw_history = deque(maxlen=160)
        self.visual_received_at = rospy.Time(0)
        self.visual_confirmations = 0
        self.last_visual_goal_time = rospy.Time(0)
        self.visual_rescued_seen_at = rospy.Time(0)
        self.coverage_cells = {}
        self.coverage_obstacle_cells = set()
        self.coverage_scan_time = rospy.Time(0)
        self.last_coverage_gain_time = rospy.Time.now()
        self.loop_trajectory = deque()
        self.loop_cooldown_until = rospy.Time(0)
        self.navigation_guard_reason = "unknown"
        self.fire_boundary_since = None
        self.fire_replan_cooldown_until = rospy.Time(0)
        self.wall_blocked_since = None
        self.wall_replan_cooldown_until = rospy.Time(0)

        self.goal_publisher = rospy.Publisher(
            "/goal_point", PointStamped, queue_size=1
        )
        self.target_publisher = rospy.Publisher(
            "/victim_mission/target", PointStamped, queue_size=1, latch=True
        )
        self.known_publisher = rospy.Publisher(
            "/victim_mission/known_victims", PoseArray, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(
            "/victim_mission/markers", MarkerArray, queue_size=1
        )
        self.active_publisher = rospy.Publisher(
            "/victim_mission/active", Bool, queue_size=1, latch=True
        )
        self.near_target_publisher = rospy.Publisher(
            "/victim_mission/near_target", Bool, queue_size=1, latch=True
        )
        self.status_publisher = rospy.Publisher(
            "/victim_mission/status", String, queue_size=1, latch=True
        )
        self.state_publisher = rospy.Publisher(
            "/victim_mission/state", String, queue_size=1, latch=True
        )
        self.rescued_count_publisher = rospy.Publisher(
            "/victim_mission/rescued_count", Int32, queue_size=1, latch=True
        )
        self.stop_publisher = rospy.Publisher(
            "/stop", Int8, queue_size=1
        )
        self.turn_rate_publisher = rospy.Publisher(
            "/victim_mission/turn_rate", Float32, queue_size=1
        )

        rospy.Subscriber(
            "/victim_detection/map_poses",
            PoseArray,
            self.detections_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_visual_bearing_topic", "/victim_detection/visual_bearing"
            ),
            Vector3Stamped,
            self.visual_bearing_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/state_estimation"),
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~victim_call_topic", "/victim_call/bearing"),
            Vector3Stamped,
            self.call_bearing_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_call_detected_topic", "/victim_call/detected"
            ),
            Bool,
            self.call_detected_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_call_strength_topic", "/victim_call/strength"
            ),
            Float32,
            self.call_strength_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_call_precise_position_topic",
                "/victim_call/estimated_position",
            ),
            PointStamped,
            self.call_precise_position_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_call_remaining_topic", "/victim_call/remaining"
            ),
            Int32,
            self.remaining_victims_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~victim_call_total_topic", "/victim_call/total"),
            Int32,
            self.total_victims_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~victim_in_rescue_range_topic",
                "/victim_call/in_rescue_range",
            ),
            Bool,
            self.in_rescue_range_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.topology_topic,
            MarkerArray,
            self.topology_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~local_recovery_active_topic", "/local_recovery/active"
            ),
            Bool,
            self.local_recovery_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/local_recovery/replan_required", Bool,
            self.replan_required_callback, queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~navigation_guard_reason_topic",
                "/navigation_safety/guard_reason",
            ),
            String,
            self.navigation_guard_reason_callback,
            queue_size=1,
        )
        rospy.Subscriber("/local_recovery/failure", String,
                         self.recovery_failure_callback, queue_size=1)
        rospy.Subscriber("/viz_path_topic", Marker, self.recovery_path_callback, queue_size=1)
        self.resume_reason_pub = rospy.Publisher("/victim_mission/route_validation", String, queue_size=5)
        rospy.Subscriber("/far_planning_result", String, self.far_result_callback, queue_size=1)
        rospy.Subscriber("/far_waypoint_guard/status", String,
                         self.far_handoff_callback, queue_size=1)
        rospy.Subscriber(
            "/local_recovery/failed_goal",
            PointStamped,
            self.local_recovery_failed_goal_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/local_recovery/failed_pocket",
            PointStamped,
            self.local_recovery_failed_pocket_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/cmd_vel_raw", TwistStamped, self.raw_command_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~victim_standoff_obstacle_topic", "/FAR_obs_debug"),
            PointCloud2, self.obstacle_cloud_callback, queue_size=1, buff_size=16777216,
        )
        rospy.Service("/victim_mission/start", Trigger, self.start_service)
        rospy.Subscriber("/registered_scan", PointCloud2,
                         self.coverage_scan_callback, queue_size=1, buff_size=16777216)
        rospy.Service("/victim_mission/stop", Trigger, self.stop_service)
        rospy.Service(
            "/victim_mission/mark_rescued", Trigger, self.mark_rescued_service
        )
        rospy.Service("/victim_mission/reset", Empty, self.reset_service)
        rospy.Subscriber("/victim_call/scene_state", String,
                         self.scene_state_callback, queue_size=1)
        self.test_goal = None
        self.test_arrived = False
        self.test_started = None
        self.test_handed_off_to_victim = False
        self.test_logger_ready = False
        self.test_odom_received = rospy.Time(0)
        rospy.Subscriber("/experiment_logger/status", String, self.test_logger_callback, queue_size=1)
        self.test_last_publish = rospy.Time(0)
        rospy.Subscriber("/recovery_test/goal", PointStamped, self.test_goal_callback, queue_size=1)
        rospy.Subscriber("/recovery_test/arrived", Bool, self.test_arrived_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.near_target_publisher.publish(Bool(data=False))
        self.publish_state("waiting for confirmed victims", self.state)
        rospy.loginfo(
            "Victim mission manager ready: confirmation=%d rescue_distance=%.2f m",
            self.confirmation_count,
            self.rescue_distance,
        )

    def detections_callback(self, message):
        if message.header.frame_id != self.world_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring victim poses in frame %s; expected %s",
                message.header.frame_id,
                self.world_frame,
            )
            return
        now = message.header.stamp
        age = (rospy.Time.now() - now).to_sec()
        if now.to_sec() <= 0 or age < -0.25 or age > 3.0:
            return
        with self.lock:
            matched_track_ids = set()
            saw_unrescued_candidate = False
            for pose in message.poses:
                if not all(math.isfinite(value) for value in (
                        pose.position.x, pose.position.y, pose.position.z)):
                    continue
                # A rescued person may remain visible. Do not create a fresh
                # target for another observation of the same physical victim.
                suppressed_by_track = any(
                    track.rescued
                    and math.hypot(pose.position.x - track.x, pose.position.y - track.y)
                    <= self.rescued_suppression_distance
                    for track in self.tracks
                )
                suppressed_by_memory = any(
                    math.hypot(pose.position.x - x, pose.position.y - y)
                    <= self.rescued_suppression_distance
                    for x, y, _z in self.rescued_positions
                )
                if suppressed_by_track or suppressed_by_memory:
                    # Precise localization proves that the yellow object now
                    # in the camera is an already rescued victim. Suppress the
                    # bearing-only controller as well, otherwise it will keep
                    # staring at the person that the precise tracker ignores.
                    self.visual_rescued_seen_at = now
                    continue

                saw_unrescued_candidate = True

                nearest = None
                nearest_distance = float("inf")
                for track in self.tracks:
                    if track.rescued or track.track_id in matched_track_ids:
                        continue
                    distance = math.hypot(pose.position.x - track.x,
                                          pose.position.y - track.y)
                    if distance < nearest_distance:
                        nearest = track
                        nearest_distance = distance

                if nearest is not None and nearest_distance <= self.merge_distance:
                    if now <= nearest.last_seen:
                        matched_track_ids.add(nearest.track_id)
                        continue
                    alpha = self.update_alpha
                    nearest.x = (1.0 - alpha) * nearest.x + alpha * pose.position.x
                    nearest.y = (1.0 - alpha) * nearest.y + alpha * pose.position.y
                    nearest.z = (1.0 - alpha) * nearest.z + alpha * pose.position.z
                    nearest.confirmations += 1
                    nearest.last_seen = now
                    matched_track_ids.add(nearest.track_id)
                else:
                    track = VictimTrack(self.next_track_id, pose, now)
                    self.next_track_id += 1
                    self.tracks.append(track)
                    matched_track_ids.add(track.track_id)

            self.merge_duplicate_active_tracks()

            # A localized, unrescued person immediately cancels any temporary
            # recovery-exploration lease.  The normal confirmation and target
            # selection rules still apply, but exploration can no longer own
            # the goal after perception has produced a credible candidate.
            if saw_unrescued_candidate:
                self.recovery_exploration_until = rospy.Time(0)

            self.tracks = [
                track for track in self.tracks
                if track.rescued
                or track.confirmations >= self.confirmation_count
                or (now - track.last_seen).to_sec() <= self.unconfirmed_timeout
            ]

    def merge_duplicate_active_tracks(self):
        """Collapse fragmented observations of one victim into one track."""
        merged = []
        for candidate in sorted(self.tracks, key=lambda item: item.track_id):
            if candidate.rescued:
                merged.append(candidate)
                continue

            primary = next(
                (
                    track
                    for track in merged
                    if not track.rescued
                    and math.hypot(candidate.x - track.x, candidate.y - track.y)
                    <= self.merge_distance
                ),
                None,
            )
            if primary is None:
                merged.append(candidate)
                continue

            primary.x = 0.5 * (primary.x + candidate.x)
            primary.y = 0.5 * (primary.y + candidate.y)
            primary.z = 0.5 * (primary.z + candidate.z)
            primary.confirmations = max(
                primary.confirmations, candidate.confirmations
            )
            primary.last_seen = max(primary.last_seen, candidate.last_seen)
            if self.current_target_id == candidate.track_id:
                self.current_target_id = primary.track_id

        self.tracks = merged

    def odom_callback(self, message):
        self.test_odom_received = rospy.Time.now()
        now = rospy.Time.now()
        new_position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        self.robot_position = new_position
        self.update_coverage(new_position[0], new_position[1], now)
        self.update_loop_detection(new_position[0], new_position[1], now)
        if (
            not self.visited_positions
            or math.hypot(
                new_position[0] - self.visited_positions[-1][0],
                new_position[1] - self.visited_positions[-1][1],
            ) >= self.exploration_visit_spacing
        ):
            self.visited_positions.append((new_position[0], new_position[1]))
            if len(self.visited_positions) > self.exploration_history_limit:
                self.visited_positions = self.visited_positions[
                    -self.exploration_history_limit:
                ]
        if self.progress_anchor_position is None:
            self.progress_anchor_position = new_position
            self.progress_anchor_time = rospy.Time.now()
        elif self.detour_goal is None and math.hypot(
            new_position[0] - self.progress_anchor_position[0],
            new_position[1] - self.progress_anchor_position[1],
        ) >= self.stuck_progress_distance:
            self.progress_anchor_position = new_position
            self.progress_anchor_time = rospy.Time.now()
        orientation = message.pose.pose.orientation
        _, _, self.robot_yaw = self._quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        stamp = message.header.stamp
        if stamp.to_sec() > 0.0:
            self.odom_yaw_history.append((stamp.to_sec(), self.robot_yaw))

    def topology_callback(self, message):
        """Cache deduplicated FAR global vertices as reachable candidates."""
        if not self.use_topological_exploration:
            return
        nodes = {}
        for marker in message.markers:
            if marker.header.frame_id not in (self.world_frame, "/" + self.world_frame):
                continue
            if marker.ns != "global_vertex":
                continue
            for point in marker.points:
                if not (math.isfinite(point.x) and math.isfinite(point.y)):
                    continue
                if not (
                    self.map_min_x <= point.x <= self.map_max_x
                    and self.map_min_y <= point.y <= self.map_max_y
                ):
                    continue
                # FAR can publish several nearly identical contour vertices.
                # A one-metre key keeps the candidate set compact and stable.
                key = (int(round(point.x)), int(round(point.y)))
                nodes[key] = (point.x, point.y)
        if nodes:
            self.graph_nodes = list(nodes.values())

    def coverage_key(self, x, y):
        return (
            int(math.floor(x / self.coverage_cell_size)),
            int(math.floor(y / self.coverage_cell_size)),
        )

    def update_loop_detection(self, x, y, now):
        """Detect travelled closed loops, not merely a stationary vehicle."""
        if (self.state != "AUTONOMOUS_EXPLORATION" or not self.active
                or self.current_target_id is not None or self.visual_guidance_active
                or self.local_recovery_active or self.last_call_goal_position is None):
            self.loop_trajectory.clear()
            self._loop_goal = None
            return
        if getattr(self, "_loop_goal", None) != self.last_call_goal_position:
            self.loop_trajectory.clear()
            self._loop_goal = self.last_call_goal_position
        if (
            self.loop_trajectory
            and math.hypot(x - self.loop_trajectory[-1][1], y - self.loop_trajectory[-1][2])
            < 0.35
        ):
            return
        self.loop_trajectory.append((now, x, y))
        cutoff = now - rospy.Duration(self.loop_detection_window)
        while self.loop_trajectory and self.loop_trajectory[0][0] < cutoff:
            self.loop_trajectory.popleft()
        if (
            now < self.loop_cooldown_until
            or not self.active
            or self.current_target_id is not None
            or self.local_recovery_active
            or self.visual_guidance_active
            or self.last_call_goal_position is None
            or len(self.loop_trajectory) < 4
        ):
            return

        samples = list(self.loop_trajectory)
        travelled = 0.0
        # Walk backward along the recent trajectory. A long travelled path
        # returning close to an older pose is the characteristic "rings"
        # failure even though odometry reports continuous movement.
        newer_x, newer_y = x, y
        for sample_time, old_x, old_y in reversed(samples[:-1]):
            travelled += math.hypot(newer_x - old_x, newer_y - old_y)
            newer_x, newer_y = old_x, old_y
            age = (now - sample_time).to_sec()
            if (
                age >= self.loop_min_duration
                and travelled >= self.loop_min_path_length
                and math.hypot(x - old_x, y - old_y) <= self.loop_closure_radius
            ):
                failed = self.last_call_goal_position
                self.remember_failed_goal(failed)
                self.last_call_goal_position = None
                self.last_call_goal_time = rospy.Time(0)
                self.detour_goal = None
                self.detour_locked_side = None
                self.loop_trajectory.clear()
                self.loop_cooldown_until = now + rospy.Duration(
                    self.loop_detection_cooldown
                )
                self.last_coverage_gain_time = now
                rospy.logwarn(
                    "Closed exploration loop detected (%.1fm/%.1fs); blacklisted goal (%.2f, %.2f)",
                    travelled, age, failed[0], failed[1]
                )
                break

    def update_coverage(self, robot_x, robot_y, now):
        """Record visited space; visible space is added by measured scan rays."""
        observed = [(robot_x, robot_y)]
        gained = False
        for x, y in observed:
            key = self.coverage_key(x, y)
            if key not in self.coverage_cells:
                gained = True
            self.coverage_cells[key] = now
        if gained:
            self.last_coverage_gain_time = now

    def coverage_scan_callback(self, message):
        # A conservative 2-D visibility estimate, not an occupancy SLAM map.
        # Only measured horizontal obstacle returns establish free rays.
        if message.header.frame_id.lstrip("/") != self.world_frame.lstrip("/"):
            return
        now = rospy.Time.now()
        with self.lock:
            if self.robot_position is None or (now - self.coverage_scan_time).to_sec() < 0.5:
                return
            age = (now - message.header.stamp).to_sec()
            if age < -0.1 or age > 0.5:
                return
            self.coverage_scan_time = now
            rx, ry, rz = self.robot_position
            rays = {}
            try:
                for x, y, z in pc2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
                    height = z - (rz - self.vehicle_height)
                    if not 0.2 <= height <= 1.8:
                        continue
                    dx, dy = x - rx, y - ry
                    distance = math.hypot(dx, dy)
                    if not math.isfinite(distance) or distance < 0.2:
                        continue
                    bucket = int(math.floor(math.atan2(dy, dx) / math.radians(0.5)))
                    if bucket not in rays or distance < rays[bucket][0]:
                        rays[bucket] = (distance, dx / distance, dy / distance)
            except (TypeError, ValueError):
                return
            gained = False
            limit = self.coverage_gain_radius * 2.0
            for distance, ux, uy in rays.values():
                if distance <= limit:
                    self.coverage_obstacle_cells.add(self.coverage_key(rx + distance * ux, ry + distance * uy))
                # Never extrapolate past a return or infer unseen angles.
                for step in range(int(min(distance, limit) / 0.5)):
                    x, y = rx + step * 0.5 * ux, ry + step * 0.5 * uy
                    if not (self.map_min_x <= x <= self.map_max_x and self.map_min_y <= y <= self.map_max_y):
                        continue
                    key = self.coverage_key(x, y)
                    if key not in self.coverage_cells:
                        gained = True
                    self.coverage_cells[key] = now
            if gained:
                self.last_coverage_gain_time = now

    def coverage_gain(self, x, y):
        center_x, center_y = self.coverage_key(x, y)
        cells = int(math.ceil(self.coverage_gain_radius / self.coverage_cell_size))
        total = 0
        unknown = 0
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if math.hypot(dx, dy) * self.coverage_cell_size > self.coverage_gain_radius:
                    continue
                key = (center_x + dx, center_y + dy)
                wx, wy = (key[0] + 0.5) * self.coverage_cell_size, (key[1] + 0.5) * self.coverage_cell_size
                if not (self.map_min_x <= wx <= self.map_max_x and self.map_min_y <= wy <= self.map_max_y):
                    continue
                # Exclude occupied cells and unknown cells hidden behind known
                # walls from a candidate's expected visibility gain.
                span = max(abs(dx), abs(dy), 1)
                if any((center_x + round(dx * i / span), center_y + round(dy * i / span))
                       in self.coverage_obstacle_cells for i in range(1, span + 1)):
                    continue
                total += 1
                if (center_x + dx, center_y + dy) not in self.coverage_cells:
                    unknown += 1
        return float(unknown) / max(1, total)

    def nearest_visited_distance(self, x, y):
        if not self.visited_positions:
            return float("inf")
        return min(
            math.hypot(x - visited_x, y - visited_y)
            for visited_x, visited_y in self.visited_positions
        )

    def remember_failed_goal(self, goal):
        if goal is None:
            return
        self.failed_exploration_goals.append(
            (goal[0], goal[1], rospy.Time.now())
        )
        if len(self.failed_exploration_goals) > self.exploration_failed_goal_limit:
            self.failed_exploration_goals = self.failed_exploration_goals[
                -self.exploration_failed_goal_limit:
            ]

    def is_failed_goal(self, x, y):
        now = rospy.Time.now()
        self.failed_exploration_goals = [
            item for item in self.failed_exploration_goals
            if len(item) < 3
            or (now - item[2]).to_sec() < self.exploration_failed_goal_cooldown
        ]
        return any(
            math.hypot(x - item[0], y - item[1])
            < self.exploration_failed_goal_radius
            for item in self.failed_exploration_goals
        )

    def remember_failed_pocket(self, pocket):
        if pocket is None:
            return
        now = rospy.Time.now()
        for index, item in enumerate(self.failed_exploration_pockets):
            if math.hypot(pocket[0] - item[0], pocket[1] - item[1]) <= (
                self.exploration_failed_pocket_merge_distance
            ):
                self.failed_exploration_pockets[index] = (
                    0.7 * item[0] + 0.3 * pocket[0],
                    0.7 * item[1] + 0.3 * pocket[1],
                    now,
                )
                return
        self.failed_exploration_pockets.append((pocket[0], pocket[1], now))
        if len(self.failed_exploration_pockets) > self.exploration_failed_goal_limit:
            self.failed_exploration_pockets = self.failed_exploration_pockets[
                -self.exploration_failed_goal_limit:
            ]

    def active_failed_pockets(self):
        now = rospy.Time.now()
        self.failed_exploration_pockets = [
            item for item in self.failed_exploration_pockets
            if (now - item[2]).to_sec() < self.exploration_failed_pocket_cooldown
        ]
        return self.failed_exploration_pockets

    def route_crosses_failed_pocket(self, goal_x, goal_y):
        """Reject targets whose direct departure corridor repeats a failed route."""
        if self.robot_position is None:
            return False
        start_x, start_y = self.robot_position[:2]
        segment_x = goal_x - start_x
        segment_y = goal_y - start_y
        length_sq = segment_x * segment_x + segment_y * segment_y
        if length_sq < 1.0e-6:
            return False
        for pocket_x, pocket_y, _stamp in self.active_failed_pockets():
            start_distance = math.hypot(start_x - pocket_x, start_y - pocket_y)
            goal_distance = math.hypot(goal_x - pocket_x, goal_y - pocket_y)
            if goal_distance < self.exploration_failed_pocket_radius:
                return True
            # If trace-back has not yet cleared the pocket, allow targets that
            # lead out instead of rejecting every possible escape direction.
            if start_distance < self.exploration_failed_pocket_radius:
                continue
            projection = (
                (pocket_x - start_x) * segment_x
                + (pocket_y - start_y) * segment_y
            ) / length_sq
            if not 0.05 <= projection <= 0.95:
                continue
            closest_x = start_x + projection * segment_x
            closest_y = start_y + projection * segment_y
            if math.hypot(closest_x - pocket_x, closest_y - pocket_y) < (
                self.exploration_failed_pocket_radius
            ):
                return True
        return False

    def recovery_clearance_goal(self, node, resume):
        candidates = self.recovery_clearance_candidates(node, resume)
        return candidates[0] if candidates else None

    def endpoint_clear(self, x, y, radius):
        grid = getattr(self, "obstacle_grid", None)
        if grid is None:
            return not any((x-ox)**2 + (y-oy)**2 < radius**2
                           for ox, oy in self.obstacle_points)
        for ix in range(math.floor((x-radius)/2.0), math.floor((x+radius)/2.0)+1):
            for iy in range(math.floor((y-radius)/2.0), math.floor((y+radius)/2.0)+1):
                if any((x-ox)**2 + (y-oy)**2 < radius**2
                       for ox, oy in grid.get((ix, iy), ())):
                    return False
        return True

    def exploration_corridor_clear(self, x, y):
        return self.observed_segment_clear(self.robot_position[:2], (x, y))

    def observed_segment_clear(self, start, end):
        rx, ry = start
        x, y = end
        distance = math.hypot(x-rx, y-ry)
        steps = max(1, int(math.ceil(distance/0.5)))
        # Add half the sample spacing: the union covers the entire swept disc.
        radius = self.recovery_goal_clearance - 1.5 + 0.25
        return all(self.endpoint_clear(rx+(x-rx)*i/steps, ry+(y-ry)*i/steps, radius)
                   for i in range(steps+1))

    def recovery_clearance_candidates(self, node, resume, selection_time=None):
        """Offset contour vertices; never treat an obstacle vertex as a goal.

        This checks endpoint clearance only. FAR and the local safety arbiter
        still validate the route and executed motion.
        """
        age = ((selection_time or rospy.Time.now()) - getattr(
            self, "obstacle_points_stamp", rospy.Time(0))).to_sec()
        if not self.obstacle_points or not 0.0 <= age <= 1.5:
            return []
        # Circumscribed chassis radius plus margin and arrival tolerance.
        radius = self.recovery_goal_clearance
        candidates = [node]
        for i in range(16):
            angle = i * 2.0 * math.pi / 16
            candidates.append((node[0] + radius * math.cos(angle),
                               node[1] + radius * math.sin(angle)))
        valid = []
        for x, y in candidates:
            if not (self.map_min_x <= x <= self.map_max_x and
                    self.map_min_y <= y <= self.map_max_y):
                continue
            if not self.endpoint_clear(x, y, radius):
                continue
            cost = math.hypot(x-self.robot_position[0], y-self.robot_position[1])
            cost += math.hypot(x-resume[0], y-resume[1])
            valid.append((cost, x, y))
        return [(x, y) for _, x, y in sorted(valid)]

    def select_topological_goal(
        self, direction_x, direction_y, side=None, unconstrained=False, target_hint=None
    ):
        """Pick a reachable, unvisited V-Graph node in the desired sector."""
        if (
            not self.use_topological_exploration
            or self.robot_position is None
            or not (self.graph_nodes or self.coverage_cells)
        ):
            return None

        now = rospy.Time.now()

        def choose(enforce_cooldown):
            resume = None
            if self.replan_required or getattr(self, "test_intermediate", None) is not None:
                resume = getattr(self, "recovery_resume_goal", None)
                if getattr(self, "test_goal", None) is not None:
                    resume = (self.test_goal.point.x, self.test_goal.point.y)
            best_goal = None
            best_score = -float("inf")
            ranked_goals = []
            candidates = []
            counts = dict(nodes=len(self.graph_nodes), no_clearance=0,
                          unchanged=0, failed_goal=0, failed_pocket=0, distance=0,
                          direction=0, cooldown=0, accepted=0)
            counts['cloud_age']=round((now-getattr(self,'obstacle_points_stamp',rospy.Time(0))).to_sec(),2)
            counts['coverage_cells']=len(self.coverage_cells)
            for node in self.graph_nodes:
                offsets = self.recovery_clearance_candidates(
                    node, resume if resume is not None else self.robot_position[:2], now)
                if not offsets:
                    counts["no_clearance"] += 1
                candidates.extend(offsets)
            # Contour vertices alone can disappear in a fully observed room.
            # Add measured free-space cell centres, allowing travel back through
            # known space towards an unseen room. No scene/victim coordinates.
            if resume is None:
                radius = self.recovery_goal_clearance
                obstacle_age = (now - getattr(self, "obstacle_points_stamp", rospy.Time(0))).to_sec()
                if self.obstacle_points and 0.0 <= obstacle_age <= 1.5:
                    for cx, cy in self.coverage_cells:
                        x = (cx + 0.5) * self.coverage_cell_size
                        y = (cy + 0.5) * self.coverage_cell_size
                        if not (self.map_min_x <= x <= self.map_max_x and
                                self.map_min_y <= y <= self.map_max_y):
                            continue
                        if not self.endpoint_clear(x, y, radius):
                            continue
                        candidates.append((x, y))
            for node_x, node_y in dict.fromkeys(candidates):
                route_retry = getattr(self, '_route_retry_after', {}).get((node_x,node_y))
                if route_retry is not None and now < route_retry:
                    continue
                if not self.goal_is_new_after_recovery(node_x, node_y):
                    counts["unchanged"] += 1
                    continue
                if self.is_failed_goal(node_x, node_y):
                    counts["failed_goal"] += 1
                    continue
                if self.route_crosses_failed_pocket(node_x, node_y):
                    counts["failed_pocket"] += 1
                    continue
                delta_x = node_x - self.robot_position[0]
                delta_y = node_y - self.robot_position[1]
                distance = math.hypot(delta_x, delta_y)
                if not (
                    self.exploration_node_min_distance
                    <= distance
                    <= self.exploration_node_max_distance
                ):
                    counts["distance"] += 1
                    continue

                forward_alignment = (
                    delta_x * direction_x + delta_y * direction_y
                ) / distance
                lateral_alignment = (
                    direction_x * delta_y - direction_y * delta_x
                ) / distance
                if side is None:
                    if (
                        not unconstrained
                        and forward_alignment < self.exploration_min_direction_alignment
                    ):
                        counts["direction"] += 1
                        continue
                    directional_score = (
                        0.0 if unconstrained
                        else self.exploration_direction_weight * forward_alignment
                    )
                else:
                    side_alignment = lateral_alignment * side
                    if side_alignment < 0.2:
                        counts["direction"] += 1
                        continue
                    directional_score = (
                        self.exploration_direction_weight * side_alignment
                        + 0.4 * max(-0.5, forward_alignment)
                    )

                visited_distance = self.nearest_visited_distance(node_x, node_y)
                gain = self.coverage_gain(node_x, node_y)
                if (unconstrained and resume is None and gain <= 0.0
                        and visited_distance < self.exploration_visited_radius):
                    continue
                cell_time = self.coverage_cells.get(self.coverage_key(node_x, node_y))
                recent_age = float("inf") if cell_time is None else (now - cell_time).to_sec()
                if (resume is None and enforce_cooldown
                        and visited_distance < self.exploration_visited_radius
                        and recent_age < self.coverage_region_cooldown):
                    counts["cooldown"] += 1
                    continue
                recent_penalty = (
                    self.coverage_recent_penalty
                    * max(0.0, 1.0 - recent_age / self.coverage_region_cooldown)
                    if math.isfinite(recent_age) and visited_distance < self.exploration_visited_radius else 0.0
                )
                novelty_score = min(
                    4.0, visited_distance / max(0.1, self.exploration_visited_radius)
                )
                score = (
                    directional_score
                    + self.exploration_novelty_weight * novelty_score
                    + self.exploration_distance_weight * distance
                    + self.coverage_gain_weight * gain
                    - recent_penalty
                )
                if target_hint is not None:
                    # Retain the noisy acoustic cue across obstacle detours.
                    # This ranks measured candidates, not a hidden victim pose.
                    score -= 2.0 * math.hypot(node_x-target_hint[0], node_y-target_hint[1])
                if resume is None and self.exploration_corridor_clear(node_x, node_y):
                    # Prefer a measured collision-free observation leg over a
                    # high-gain point across a wall. FAR still executes the leg.
                    score += 2.0
                if resume is not None:
                    # Recovery is not exploration: rank by geometric travel
                    # via the candidate to the retained mission target. This
                    # is a heuristic, not proof of a collision-free route.
                    score = -(distance + math.hypot(
                        node_x - resume[0], node_y - resume[1]))
                counts["accepted"] += 1
                ranked_goals.append((score, (node_x, node_y)))
                if score > best_score:
                    best_score = score
                    best_goal = (node_x, node_y)
            self.recovery_selection_diagnostic = ", ".join(
                "%s=%s" % item for item in counts.items())
            if resume is None:
                points = self.observed_route_points()
                segment_cache = {}
                def cached_clear(start, end):
                    key = tuple(sorted((tuple(start), tuple(end))))
                    if key not in segment_cache:
                        segment_cache[key] = self.observed_segment_clear(start, end)
                    return segment_cache[key]
                for _, candidate in sorted(ranked_goals, reverse=True)[:12]:
                    route = observed_route(self.robot_position[:2], candidate,
                                           points, cached_clear)
                    if route:
                        self._observed_route_goal = candidate
                        self._observed_route = route
                        return candidate
                return None
            return best_goal

        # Prefer a genuinely new region. Only relax cooldown when every
        # reachable candidate has recently been observed.
        return choose(True) or choose(False)

    def visual_bearing_callback(self, message):
        with self.lock:
            if message.header.frame_id.lstrip("/") != "vehicle":
                rospy.logwarn_throttle(
                    2.0, "Ignoring visual bearing in frame %s; expected vehicle",
                    message.header.frame_id,
                )
                return
            forward = float(message.vector.x)
            left = float(message.vector.y)
            norm = math.hypot(forward, left)
            if not math.isfinite(norm) or norm < 1e-6:
                return
            now = rospy.Time.now()
            stamp = message.header.stamp
            if (stamp.to_sec() <= 0.0 or stamp <= self.visual_last_stamp
                    or (now - stamp).to_sec() > self.visual_max_observation_age
                    or (now - stamp).to_sec() < -0.2):
                return
            # The image belongs to its capture pose. Combining an old image
            # with the current yaw makes a fixed person appear to move while
            # the robot turns and produces a moving navigation carrot.
            capture_yaw = self.yaw_at_stamp(stamp)
            if capture_yaw is None:
                return
            world_angle = capture_yaw + math.atan2(left, forward)
            previous = self.visual_world_angle
            delta = (self.angle_difference(world_angle, previous)
                     if previous is not None else 0.0)
            if ((stamp - self.visual_received_at).to_sec() <= self.visual_timeout
                    and previous is not None
                    and abs(delta) <= self.visual_direction_tolerance):
                self.visual_confirmations += 1
                world_angle = previous + 0.5 * delta
            else:
                self.visual_confirmations = 1
            self.visual_bearing = (forward / norm, left / norm)
            self.visual_world_angle = world_angle
            self.visual_received_at = stamp
            self.visual_last_stamp = stamp
            if self.visual_confirmations >= self.visual_confirmation_count:
                self.recovery_exploration_until = rospy.Time(0)

    @staticmethod
    def angle_difference(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def yaw_at_stamp(self, stamp):
        samples = tuple(self.odom_yaw_history)
        if not samples:
            return None
        value = stamp.to_sec()
        # Permit a small odometry/image transport skew, never a many-second
        # extrapolation through an unknown turn.
        if value < samples[0][0] - 0.25 or value > samples[-1][0] + 0.25:
            return None
        for index in range(1, len(samples)):
            older_time, older_yaw = samples[index - 1]
            newer_time, newer_yaw = samples[index]
            if older_time <= value <= newer_time:
                fraction = (value - older_time) / max(1e-6, newer_time - older_time)
                return older_yaw + fraction * self.angle_difference(newer_yaw, older_yaw)
        return samples[0][1] if value <= samples[0][0] else samples[-1][1]

    def visual_observation_available(self, now, allow_grace=True):
        if (self.visual_world_angle is None
                or self.visual_confirmations < self.visual_confirmation_count
                or self.robot_position is None):
            return False
        limit = (self.visual_lost_grace
                 if allow_grace and self.visual_guidance_active
                 else self.visual_timeout)
        if (now - self.visual_received_at).to_sec() > limit:
            return False
        # Suppress only a rescued object on the currently observed ray. A
        # rescued person elsewhere in the same frame must not hide a new one.
        return not any(
            math.hypot(x - self.robot_position[0], y - self.robot_position[1])
            <= self.precise_target_range
            and abs(self.angle_difference(
                math.atan2(y - self.robot_position[1], x - self.robot_position[0]),
                self.visual_world_angle,
            )) <= math.radians(10.0)
            for x, y, _z in self.rescued_positions
        )

    def clear_visual_guidance(self):
        self.visual_guidance_active = False
        self.visual_goal_position = None
        self.last_visual_goal_time = rospy.Time(0)

    def replan_required_callback(self, message):
        with self.lock:
            required = bool(message.data)
            was_required = self.replan_required
            if required and not self.replan_required:
                self.recovery_endpoint_attempts = []
                self.resume_path_probe = None
                self.recovery_resume_goal = getattr(self, "last_published_navigation_goal", None)
                self.recovery_candidate_wait_started = None
                self.test_intermediate = None
                self.call_approach_goal_position = None
                self.call_approach_goal_time = rospy.Time(0)
                self.replan_request_pending = True
            self.replan_required = required
            if not required:
                if was_required and self.last_call_goal_position is not None:
                    self.test_intermediate = self.last_call_goal_position
                    self.test_intermediate_started = rospy.Time.now()
                    self.test_intermediate_progress = rospy.Time.now()
                    self.test_intermediate_best = float("inf")
                self.rejected_navigation_goal = None
                # Waiting for a route is not failed exploration progress.
                # Start a new progress budget instead of immediately blacklisting
                # the newly accepted escape goal on the next mission tick.
                self.progress_anchor_time = rospy.Time.now()
                self.progress_anchor_position = self.robot_position
                self.last_coverage_gain_time = rospy.Time.now()

    def far_handoff_callback(self, message):
        with self.lock:
            self.far_handoff_status = message.data
            self.far_handoff_received = rospy.Time.now()

    def maintain_acoustic_goal(self, now, direction_x, direction_y):
        """Hold a route while progressing; replace a stalled FAR handoff."""
        goal = self.call_approach_goal_position
        if goal is None:
            return False
        remaining = math.hypot(goal[0] - self.robot_position[0],
                               goal[1] - self.robot_position[1])
        if remaining <= self.exploration_goal_arrival_distance:
            self.call_approach_goal_position = None
            self._acoustic_progress = None
            return False
        previous = getattr(self, "_acoustic_progress", None)
        progress_key = goal
        route = getattr(self, '_observed_route', [])
        if getattr(self, '_observed_route_goal', None) == goal and route:
            remaining = math.dist(self.robot_position[:2], route[0])
            progress_key = (goal, route[0])
        if previous is None or previous[0] != progress_key or now < previous[2]:
            previous = (progress_key, remaining, now)
        if remaining <= previous[1] - 0.5 or self.local_recovery_active:
            previous = (progress_key, min(remaining, previous[1]), now)
        self._acoustic_progress = previous
        blocked = (
            getattr(self, "far_handoff_status", "").startswith("blocked")
            and (now - getattr(self, "far_handoff_received", rospy.Time(0))).to_sec() < 3.0
        )
        deadline = min(6.0, self.coverage_stall_timeout) if blocked else self.coverage_stall_timeout
        if (now - previous[2]).to_sec() >= deadline:
            self.remember_failed_goal(goal)
            replacement = (self.select_topological_goal(direction_x, direction_y, side=1)
                           or self.select_topological_goal(direction_x, direction_y, side=-1))
            if replacement is None:
                self.call_approach_goal_position = None
                self._acoustic_progress = None
                return self.publish_autonomous_exploration_goal()
            self.call_approach_goal_position = replacement
            self._acoustic_progress = None
            self.publish_approach_goal(replacement[0], replacement[1], now)
            self.call_guidance_status = "acoustic route stalled; requesting alternate FAR node"
            rospy.logwarn("Acoustic handoff stalled: replacing %s with %s", goal, replacement)
            return True
        if (now - self.call_approach_goal_time).to_sec() >= self.call_goal_interval:
            self.publish_approach_goal(goal[0], goal[1], now)
        self.call_guidance_status = "following stable acoustic stage goal: {:.2f} m".format(remaining)
        return True

    def navigation_guard_reason_callback(self, message):
        """Track the final safety veto without confusing it with recovery state."""
        with self.lock:
            reason = (message.data or "unknown").strip()
            self.navigation_guard_reason = reason
            if reason != "fire_boundary":
                self.fire_boundary_since = None
            if reason != "wall_swept_footprint":
                self.wall_blocked_since = None

    def handle_wall_blocked_exploration(self, now):
        """Reject a persistently unsafe exploration endpoint and ask FAR again.

        This is a bounded planner handoff, not a motion-producing recovery
        state machine.  The safety guard remains authoritative and continues
        to output zero until a newly selected FAR route is safe.
        """
        if self.navigation_guard_reason != "wall_swept_footprint":
            self.wall_blocked_since = None
            return False
        if (
            not self.active
            or self.state != "AUTONOMOUS_EXPLORATION"
            or self.current_target_id is not None
            or self.call_detected
            or self.visual_guidance_active
            or self.last_call_goal_position is None
        ):
            self.wall_blocked_since = None
            return False
        if now < self.wall_replan_cooldown_until:
            return False
        if self.wall_blocked_since is None:
            self.wall_blocked_since = now
            return False
        if (now - self.wall_blocked_since).to_sec() < self.wall_block_replan_delay:
            return False

        rejected = self.last_call_goal_position
        self.remember_failed_goal(rejected)
        self.clear_committed_target()
        self.last_published_navigation_goal = None
        self.progress_anchor_position = self.robot_position
        self.progress_anchor_time = now
        self.last_coverage_gain_time = now
        self.wall_blocked_since = now
        self.wall_replan_cooldown_until = now + rospy.Duration(
            self.wall_block_replan_cooldown
        )
        selected = self.publish_autonomous_exploration_goal()
        if selected and self.last_call_goal_position is not None:
            replacement = self.last_call_goal_position
            self.publish_state(
                "wall veto persisted; switched exploration goal "
                "({:.1f},{:.1f}) -> ({:.1f},{:.1f})".format(
                    rejected[0], rejected[1], replacement[0], replacement[1]
                ),
                "AUTONOMOUS_EXPLORATION",
            )
        else:
            self.publish_state(
                "wall veto persisted; rejected unsafe exploration goal",
                "AUTONOMOUS_EXPLORATION",
            )
        return True

    def handle_fire_blocked_exploration(self, now):
        """Reject an exploration endpoint whose FAR route repeatedly hits fire.

        The safety guard must remain the final authority and keeps the vehicle
        stopped.  This method closes the feedback loop at mission level: after
        a short continuous veto it blacklists only the current exploration
        endpoint, clears the committed waypoint, and asks the normal FAR-node
        selector for another target.  Victim approach goals are deliberately
        excluded so a temporary fire veto cannot discard a confirmed victim.
        """
        if self.navigation_guard_reason != "fire_boundary":
            self.fire_boundary_since = None
            return False
        if (
            not self.active
            or self.state != "AUTONOMOUS_EXPLORATION"
            or self.current_target_id is not None
            or self.call_detected
            or self.visual_guidance_active
            or self.last_call_goal_position is None
        ):
            self.fire_boundary_since = None
            return False
        if now < self.fire_replan_cooldown_until:
            return False
        if self.fire_boundary_since is None:
            self.fire_boundary_since = now
            return False
        if (now - self.fire_boundary_since).to_sec() < self.fire_block_replan_delay:
            return False

        rejected = self.last_call_goal_position
        self.remember_failed_goal(rejected)
        self.clear_committed_target()
        self.last_published_navigation_goal = None
        self.progress_anchor_position = self.robot_position
        self.progress_anchor_time = now
        self.last_coverage_gain_time = now
        self.fire_boundary_since = now
        self.fire_replan_cooldown_until = now + rospy.Duration(
            self.fire_block_replan_cooldown
        )

        selected = self.publish_autonomous_exploration_goal()
        if selected and self.last_call_goal_position is not None:
            replacement = self.last_call_goal_position
            self.publish_state(
                "fire blocked FAR route; switched exploration goal "
                "({:.1f},{:.1f}) -> ({:.1f},{:.1f})".format(
                    rejected[0], rejected[1], replacement[0], replacement[1]
                ),
                "AUTONOMOUS_EXPLORATION",
            )
            rospy.logwarn(
                "Fire boundary blocked FAR exploration route; blacklisted "
                "(%.2f, %.2f), selected (%.2f, %.2f)",
                rejected[0], rejected[1], replacement[0], replacement[1],
            )
        else:
            self.publish_state(
                "fire blocked FAR route; old goal rejected, waiting for a new node",
                "AUTONOMOUS_EXPLORATION",
            )
            rospy.logwarn(
                "Fire boundary blocked FAR exploration route; blacklisted "
                "(%.2f, %.2f), no alternate node available yet",
                rejected[0], rejected[1],
            )
        return True

    def recovery_failure_callback(self, message):
        if not message.data:
            return
        with self.lock:
            # A fixed recovery-test goal must not be converted into a terminal
            # mission failure when one wall-side escape attempt fails. Keep the
            # test alive, clear the recovery latch, and let FAR select a fresh
            # approach from the current pose. The normal rescue mission keeps
            # the original terminal-failure behavior below.
            if ((getattr(self, "test_goal_received", None) is not None and self.test_started is not None)
                    or (self.call_detected and (self.remaining_victims is None or self.remaining_victims > 0))):
                self.local_recovery_active = False
                self.replan_required = False
                self.replan_request_pending = False
                self.replan_sent_at = rospy.Time(0)
                self.test_last_publish = rospy.Time(0)
                # Clear the latched stop written by the failed recovery
                # attempt before handing control back to pathFollower/FAR.
                self.stop_publisher.publish(Int8(data=0))
                self.publish_state(
                    "recovery attempt failed; retrying FAR route",
                    "NAVIGATION_TEST",
                )
                rospy.logwarn("Recovery attempt failed during navigation test; handing control back to FAR")
                return
            self.active = False
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("recovery failed: " + message.data, "RECOVERY_FAILED")

    def handle_replan_request(self, now):
        if not self.replan_required or self.robot_position is None:
            return False
        if (self.replan_request_pending or
                (now - getattr(self, "replan_sent_at", rospy.Time(0))).to_sec() > 8.0):
            self.rejected_navigation_goal = getattr(
                self, "last_published_navigation_goal", self.last_call_goal_position)
            if self.rejected_navigation_goal is not None:
                self.remember_failed_goal(self.rejected_navigation_goal)
            self.clear_visual_guidance()
            self.last_call_goal_position = None
            self.current_target_id = None
            self.detour_goal = None
            goal = self.select_topological_goal(
                math.cos(self.robot_yaw), math.sin(self.robot_yaw), unconstrained=True)
            if goal is None:
                goal = self.fallback_replan_goal(now)
            self.replan_request_pending = False
            self.replan_sent_at = now
            if goal is not None:
                self.last_call_goal_position = goal
                self.publish_call_goal(goal[0], goal[1], now)
                self.recovery_exploration_until = now + rospy.Duration(8.0)
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))
        self.near_target_publisher.publish(Bool(data=False))
        detail = ("replacement goal sent; waiting for safe handoff"
                  if self.last_call_goal_position is not None else
                  "no replacement candidate: " + getattr(
                      self, "recovery_selection_diagnostic", "topology unavailable"))
        self.publish_state(detail, "REPLANNING")
        return True

    def fallback_replan_goal(self, now):
        """Create a bounded side-step goal when the graph has no candidate.

        This is deliberately based on the live victim bearing and robot pose,
        not an obstacle name or test-map layout. Repeated attempts alternate
        sides so the controller cannot rapidly recreate the same failed goal.
        """
        if (not self.call_detected or self.call_bearing is None
                or self.robot_position is None
                or not hasattr(self, "robot_yaw")
                or (now - self.call_received_at).to_sec() > self.call_timeout):
            return None
        forward, left = self.call_bearing
        world_x = math.cos(self.robot_yaw) * forward - math.sin(self.robot_yaw) * left
        world_y = math.sin(self.robot_yaw) * forward + math.cos(self.robot_yaw) * left
        length = math.hypot(world_x, world_y)
        if length < 1.0e-3:
            return None
        world_x /= length
        world_y /= length
        lateral = max(4.0, min(10.0, self.call_goal_distance))
        forward_step = min(3.0, max(1.5, 0.3 * lateral))
        side = self.replan_fallback_side
        self.replan_fallback_side *= -1.0
        goal = (
            self.robot_position[0] + forward_step * world_x - side * lateral * world_y,
            self.robot_position[1] + forward_step * world_y + side * lateral * world_x,
        )
        rospy.logwarn(
            "No topological replacement goal; using bounded %s-side detour (%.2f, %.2f)",
            "left" if side > 0.0 else "right", goal[0], goal[1])
        return goal

    def goal_is_new_after_recovery(self, x, y):
        return not (
            self.replan_required and self.rejected_navigation_goal is not None
            and math.hypot(x - self.rejected_navigation_goal[0],
                           y - self.rejected_navigation_goal[1])
            < self.replan_goal_min_distance
        )

    @staticmethod
    def _quaternion_to_euler(x, y, z, w):
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return roll, pitch, yaw

    def call_bearing_callback(self, message):
        if not self.enable_simulated_call_guidance:
            return
        if message.header.frame_id not in ("vehicle", "/vehicle"):
            rospy.logwarn_throttle(
                2.0, "Ignoring victim call in frame %s", message.header.frame_id
            )
            return
        magnitude = math.hypot(message.vector.x, message.vector.y)
        if magnitude < 1e-6:
            return
        self.call_bearing = (
            message.vector.x / magnitude,
            message.vector.y / magnitude,
        )
        self.call_received_at = rospy.Time.now()

    def clear_committed_target(self):
        self.resume_path_probe = None
        self.test_intermediate = None
        self.recovery_resume_goal = None
        self.recovery_candidate_wait_started = None
        self.call_approach_goal_position = None
        self.call_approach_goal_time = rospy.Time(0)
        self.last_call_goal_position = None
        self.last_call_goal_time = rospy.Time(0)
        self.last_goal_position = None
        self.detour_goal = None
        self.best_call_distance = None
        self.recovery_exploration_until = rospy.Time(0)
        self.call_turn_started_at = None

    def scene_state_callback(self, message):
        try:
            episode, total_text, rescued_text, call_id = message.data.split("|")
            total = int(total_text)
            rescued = set(filter(None, rescued_text.split(",")))
            if not episode or total < 0 or len(rescued) > total:
                return
        except (ValueError, AttributeError):
            return
        with self.lock:
            new_episode = episode != self.scene_episode
            newly_rescued = rescued - self.scene_rescued_ids if not new_episode else set()
            if new_episode or newly_rescued or (
                    self.enable_simulated_call_guidance and
                    call_id and call_id != self.scene_call_id):
                self.clear_committed_target()
                self._acoustic_memory = None
                self._acoustic_scan_yield_until = 0.0
            if new_episode:
                self.tracks.clear()
                self.current_target_id = None
                self.test_handed_off_to_victim = False
            if newly_rescued:
                # Unity IDs are not perception track IDs: do not mark a guessed
                # track rescued. Reacquire observations after the interaction.
                self.tracks.clear()
                self.current_target_id = None
                # scene_state is the authoritative physical-rescue event. Do
                # not wait for a subsequent in_rescue_range=False message;
                # that message can be the one dropped during ROS-TCP queue
                # pressure, leaving the manager stuck in RESCUING forever.
                self.in_rescue_range = False
                self.rescue_zone_armed = True
                self.call_contact_started_at = None
                self.near_target_publisher.publish(Bool(data=False))
                self.stop_publisher.publish(Int8(data=0))
            self.scene_episode = episode
            self.scene_rescued_ids = rescued
            # Silence does not identify a different caller. Keep only the
            # previously received identity, never use it as a target position.
            self.scene_call_id = ((call_id or self.scene_call_id)
                                  if self.enable_simulated_call_guidance else None)
            self.expected_victim_count = total
            self.remaining_victims = total - len(rescued)
            self.call_rescued_count = len(rescued)
            self.contact_rescue_pending_remaining = None
            self.rescued_count_publisher.publish(Int32(data=len(rescued)))

    def call_detected_callback(self, message):
        if not self.enable_simulated_call_guidance:
            return
        memory = getattr(self, '_acoustic_memory', None)
        remembered = memory is not None and 0 <= rospy.Time.now().to_sec()-memory[1] <= 60.0
        if message.data and not self.call_detected and not remembered:
            self.clear_committed_target()
        self.call_detected = message.data
        if not message.data:
            self.call_bearing = None
            self.call_strength = None
            self.call_precise_position = None
            self.call_precise_received_at = rospy.Time(0)
            self.call_turn_started_at = None

    def call_strength_callback(self, message):
        if not self.enable_simulated_call_guidance:
            return
        self.call_strength = max(0.0, min(1.0, message.data))

    def call_precise_position_callback(self, message):
        if not self.enable_simulated_call_guidance:
            return
        if message.header.frame_id not in (self.world_frame, "/" + self.world_frame):
            return
        if not all(math.isfinite(value) for value in (
                message.point.x, message.point.y, message.point.z)):
            return
        with self.lock:
            self.call_precise_position = (
                float(message.point.x),
                float(message.point.y),
                float(message.point.z),
            )
            self.call_precise_received_at = rospy.Time.now()
            self._acoustic_memory = (self.call_precise_position, self.call_precise_received_at.to_sec())

    def local_recovery_callback(self, message):
        """Pause high-level detours while local obstacle recovery is active."""
        with self.lock:
            self.local_recovery_active = bool(message.data)
            if self.local_recovery_active:
                now = rospy.Time.now()
                self.call_progress_time = now
                self.progress_anchor_time = now

    def raw_command_callback(self, message):
        with self.lock:
            self.raw_linear_speed = float(message.twist.linear.x)
            self.raw_angular_speed = float(message.twist.angular.z)
            self.raw_command_received_at = rospy.Time.now()

    def obstacle_cloud_callback(self, message):
        if message.header.frame_id not in (self.world_frame, "/" + self.world_frame):
            return
        age=(rospy.Time.now()-message.header.stamp).to_sec()
        if not -0.1 <= age <= 1.5:
            rospy.logwarn_throttle(10.0, 'Dropping queued obstacle cloud age %.2fs', age)
            return
        points = []
        try:
            for index, point in enumerate(pc2.read_points(
                    message, field_names=("x", "y"), skip_nans=True)):
                points.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return
        grid = {}
        for x, y in points:
            grid.setdefault((math.floor(x/2.0), math.floor(y/2.0)), []).append((x, y))
        with self.lock:
            self.obstacle_points = points
            self.obstacle_grid = grid
            self.obstacle_points_stamp = message.header.stamp

    def standoff_candidate_is_clear(self, track, index):
        if not self.standoff_angle_offsets:
            return True
        base = self.standoff_base_angle
        if base is None:
            return True
        angle = base + self.standoff_angle_offsets[index]
        x = track.x + self.navigation_standoff_distance * math.cos(angle)
        y = track.y + self.navigation_standoff_distance * math.sin(angle)
        if not (self.map_min_x <= x <= self.map_max_x
                and self.map_min_y <= y <= self.map_max_y):
            return False
        clearance_sq = self.standoff_obstacle_clearance ** 2
        return not any(
            (x - ox) ** 2 + (y - oy) ** 2 < clearance_sq
            for ox, oy in self.obstacle_points
        )

    def advance_standoff_candidate(self, track, reason):
        count = len(self.standoff_angle_offsets)
        if count == 0:
            return False
        for step in range(1, count + 1):
            candidate = (self.standoff_candidate_index + step) % count
            if candidate in self.standoff_attempted_indices:
                continue
            self.standoff_attempted_indices.add(candidate)
            if self.standoff_candidate_is_clear(track, candidate):
                self.standoff_candidate_index = candidate
                self.standoff_progress_time = rospy.Time.now()
                self.standoff_zero_since = None
                self.last_goal_time = rospy.Time(0)
                rospy.logwarn(
                    "Victim %d: %s; switching to clear standoff %d/%d",
                    track.track_id, reason, candidate + 1, count,
                )
                return True
        track.deferred_until = rospy.Time.now() + rospy.Duration(
            self.standoff_defer_seconds
        )
        self.current_target_id = None
        self.last_goal_position = None
        self.standoff_zero_since = None
        rospy.logwarn(
            "Victim %d: all standoff candidates failed; deferred for %.0f s",
            track.track_id, self.standoff_defer_seconds,
        )
        return False

    def local_recovery_failed_goal_callback(self, message):
        """Blacklist the high-level target rejected by local recovery."""
        with self.lock:
            failed = (message.point.x, message.point.y)
            self.rejected_navigation_goal = failed
            self.remember_failed_goal(failed)
            if (
                self.last_call_goal_position is not None
                and math.hypot(
                    failed[0] - self.last_call_goal_position[0],
                    failed[1] - self.last_call_goal_position[1],
                ) <= self.exploration_failed_goal_radius
            ):
                self.last_call_goal_position = None
                self.last_call_goal_time = rospy.Time(0)
            if (
                self.detour_goal is not None
                and math.hypot(
                    failed[0] - self.detour_goal[0],
                    failed[1] - self.detour_goal[1],
                ) <= self.exploration_failed_goal_radius
            ):
                self.detour_goal = None
                self.detour_best_remaining = None
            now = rospy.Time.now()
            self.call_progress_time = now
            self.progress_anchor_position = self.robot_position
            self.progress_anchor_time = now
            rospy.logwarn(
                "Blacklisted rejected navigation target at (%.2f, %.2f)",
                failed[0],
                failed[1],
            )

    def local_recovery_failed_pocket_callback(self, message):
        """Remember the physical corner so another goal cannot reuse its route."""
        with self.lock:
            pocket = (message.point.x, message.point.y)
            self.remember_failed_pocket(pocket)
            rospy.logwarn(
                "Remembered physical failed corridor at (%.2f, %.2f)",
                pocket[0], pocket[1],
            )

    def remaining_victims_callback(self, message):
        if self.scene_episode is not None:
            return  # Count and identity come from one atomic Unity snapshot.
        previous = self.remaining_victims
        self.remaining_victims = max(0, message.data)
        unity_confirmed = max(
            0, self.expected_victim_count - self.remaining_victims
        )
        if self.active and self.mission_started_at is None:
            self.mission_started_at = rospy.Time.now()
        if previous is not None and self.remaining_victims < previous:
            self.best_call_distance = None
            self.call_progress_time = rospy.Time.now()
            self.detour_goal = None
            self.detour_locked_side = None
            self.detour_side_lock_until = rospy.Time(0)
        if (
            self.contact_rescue_pending_remaining is not None
            and self.remaining_victims < self.contact_rescue_pending_remaining
        ):
            self.contact_rescue_pending_remaining = None
            self.call_rescued_count = unity_confirmed
        elif self.contact_rescue_pending_remaining is None:
            # Outside the short ROS->Unity acknowledgement window, Unity's
            # physical victim lifecycle is authoritative. This also clears a
            # stale persisted count when a fresh scene starts with all victims.
            self.call_rescued_count = unity_confirmed

    def in_rescue_range_callback(self, message):
        with self.lock:
            self.in_rescue_range = bool(message.data)
            if not self.in_rescue_range:
                self.call_contact_started_at = None
                # A new rescue may only begin after Unity has reported at
                # least one out-of-range sample. This prevents the previous
                # victim's final True sample being counted again after its
                # remaining-count acknowledgement arrives.
                self.rescue_zone_armed = True

    def total_victims_callback(self, message):
        if self.scene_episode is not None:
            return
        if message.data <= 0:
            return
        with self.lock:
            self.expected_victim_count = int(message.data)
            if (
                self.remaining_victims is not None
                and self.contact_rescue_pending_remaining is None
            ):
                self.call_rescued_count = max(
                    0,
                    self.expected_victim_count - self.remaining_victims,
                )

    def confirmed_tracks(self):
        return [
            track for track in self.tracks
            if track.confirmations >= self.confirmation_count
        ]

    def nearest_available_track(self):
        if self.robot_position is None:
            return None
        available = [
            track
            for track in self.confirmed_tracks()
            if not track.rescued
            and rospy.Time.now() >= track.deferred_until
            and math.hypot(
                track.x - self.robot_position[0],
                track.y - self.robot_position[1],
            ) <= self.precise_target_range
        ]
        if not available:
            return None
        now = rospy.Time.now()
        visual_available = self.visual_observation_available(now)
        if visual_available:
            aligned = [track for track in available if abs(self.angle_difference(
                math.atan2(track.y - self.robot_position[1],
                           track.x - self.robot_position[0]),
                self.visual_world_angle,
            )) <= self.visual_direction_tolerance]
            locked = next((track for track in available
                           if track.track_id == self.current_target_id), None)
            # Keep a fresh precise approach. A stale remembered person in a
            # different direction cannot outrank a newly confirmed visual ray.
            if (locked is not None
                    and (now - locked.last_seen).to_sec() <= self.visual_track_freshness):
                return locked
            if not aligned:
                return None
            available = aligned
        # Once a confirmed victim has been selected, keep that target while
        # it remains valid. Choosing the nearest track on every timer tick
        # previously made the mission jump between victims and eventually
        # defer all of them without completing an approach.
        locked = next(
            (
                track for track in available
                if track.track_id == self.current_target_id
            ),
            None,
        )
        if locked is not None:
            return locked
        return min(
            available,
            key=lambda track: math.hypot(
                track.x - self.robot_position[0],
                track.y - self.robot_position[1],
            ),
        )

    def test_goal_callback(self, msg):
        with self.lock:
            # Unity republishes the fixed test goal continuously. Once a real
            # victim call owns this episode, accepting those repeats would
            # bounce between NAVIGATION_TEST and CALL_GUIDANCE every tick.
            if self.test_handed_off_to_victim:
                return
            first_goal = self.test_goal is None
            self.test_goal = msg
            self.test_goal_received = rospy.Time.now()
            if first_goal:
                self.clear_committed_target()

    def test_logger_callback(self, msg):
        self.test_logger_ready = msg.data.startswith("recording:")

    def test_arrived_callback(self, msg):
        self.test_arrived = bool(msg.data)
        self.test_arrived_received = rospy.Time.now()

    def run_navigation_test(self, now):
        fresh = (now - self.test_goal_received).to_sec() < 1.0
        ready = (fresh and self.robot_position is not None
                 and (now - self.test_odom_received).to_sec() < 1.0
                 and self.test_logger_ready)
        if not ready:
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("navigation test waiting for fresh odometry, goal and recording",
                               "TEST_WAIT")
            return
        if self.test_started is None:
            self.test_started = now
        arrived = (self.test_arrived and
                   (now - self.test_arrived_received).to_sec() < 1.0)
        # Fixed-goal scenes can also contain a real victim. Once the Unity
        # sensor reports a call, hand control to the normal rescue selector so
        # the test does not drive past the victim until its 120 s timeout.
        if self.call_detected and (self.remaining_victims is None or self.remaining_victims > 0):
            self.test_handed_off_to_victim = True
            self.test_goal = None
            self.test_started = None
            self.test_arrived = False
            self.test_arrived_received = rospy.Time(0)
            self.publish_state("victim call detected; switching to rescue guidance", "CALL_GUIDANCE")
            rospy.loginfo("Navigation test handing off to victim guidance")
            return
        if arrived or (now - self.test_started).to_sec() >= 120.0:
            self.active = False
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("navigation test reached goal" if arrived else "mission timeout: navigation test",
                               "COMPLETE" if arrived else "TIMEOUT")
            return
        if not fresh:
            self.stop_publisher.publish(Int8(data=2))
            self.publish_state("navigation test input stale", "TEST_WAIT")
            return
        if self.handle_replan_request(now) or self.local_recovery_active:
            return
        if self.follow_test_intermediate(now):
            return
        self.stop_publisher.publish(Int8(data=0))
        self.turn_rate_publisher.publish(Float32(data=0.0))
        if (now - self.test_last_publish).to_sec() >= 1.0:
            self.publish_call_goal(self.test_goal.point.x, self.test_goal.point.y, now)
            self.test_last_publish = now
        # The Unity test scene contains a real victim. Keep the fixed goal for
        # repeatable navigation, but do not claim that victim sensing is off;
        # VictimCallSensor continues publishing the acoustic/visual topics.
        self.publish_state("fixed-goal navigation test with victim call sensing", "NAVIGATION_TEST")

    def recovery_path_callback(self, msg):
        with self.lock:
            if msg.ns != "global_path" or msg.header.frame_id.lstrip("/") != self.world_frame.lstrip("/"):
                return
            self.recovery_path = (msg.header.stamp, [(p.x, p.y) for p in msg.points]
                                  if msg.action == 0 and msg.type == 4 else [])

    def path_repeats_failed_pocket(self, points):
        # Test every segment, not the straight line to the final goal.
        for a, b in zip(points, points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = dx*dx + dy*dy
            for x, y, _ in self.active_failed_pockets():
                t = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length)) if length else 0.0
                if math.hypot(a[0]+t*dx-x, a[1]+t*dy-y) < self.exploration_failed_pocket_radius:
                    return True
        return False

    def far_result_callback(self, msg):
        try:
            stamp, status, gx, gy, ax, ay = msg.data.split("|")
            stamp, gx, gy, ax, ay = map(float, (stamp, gx, gy, ax, ay))
            if not all(math.isfinite(v) for v in (stamp, gx, gy, ax, ay)):
                return
        except (TypeError, ValueError):
            return
        with self.lock:
            self.far_route_result = (stamp, status, (gx, gy), (ax, ay))
            # Scope endpoint memory to this recovery episode. Repeated result
            # messages for one request are not new attempts.
            requested = getattr(self, "last_published_navigation_goal", None)
            if (self.replan_required and status == "PATH_READY"
                    and requested is not None
                    and 0 <= rospy.Time.now().to_sec() - stamp <= 1.5
                    and stamp >= getattr(self, "replan_sent_at", rospy.Time(0)).to_sec()
                    and math.hypot(gx-requested[0], gy-requested[1]) < 0.1):
                attempts = getattr(self, "recovery_endpoint_attempts", [])
                if not any(math.hypot(gx-rx, gy-ry) < 0.1
                           for rx, ry, _, _ in attempts):
                    repeated = any(math.hypot(ax-ex, ay-ey) < 0.75
                                   for _, _, ex, ey in attempts)
                    attempts.append((gx, gy, ax, ay))
                    self.recovery_endpoint_attempts = attempts[-32:]
                    if repeated:
                        self.remember_failed_goal((gx, gy))
                        self.replan_request_pending = True
                        self.publish_state("replacement rejected: repeated FAR adjusted endpoint",
                                           "REPLANNING")

    def validate_resume_path(self, resume, now):
        """Return None while waiting, False if rejected, True if validated."""
        probe = getattr(self, "resume_path_probe", None)
        if probe is None or probe[0] != resume:
            self.resume_path_probe = (resume, now)
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_call_goal(resume[0], resume[1], now)
            return None
        stamp, points = getattr(self, "recovery_path", (rospy.Time(0), []))
        endpoint = resume
        result = getattr(self, "far_route_result", None)
        if (result is not None and result[1] == "PATH_READY"
                and result[0] > probe[1].to_sec()
                and 0 <= now.to_sec()-result[0] < 1.5
                and abs(result[0]-stamp.to_sec()) < 0.3
                and math.hypot(result[2][0]-resume[0], result[2][1]-resume[1]) < 0.1):
            endpoint = result[3]
        # Only the path endpoint changes here; Unity/original-goal completion
        # and all obstacle/failed-route checks remain unchanged.
        valid = (stamp > probe[1] and 0 <= (now-stamp).to_sec() < 1.5
                 and len(points) >= 2
                 and all(math.isfinite(v) for p in points for v in p)
                 and math.hypot(points[0][0]-self.robot_position[0], points[0][1]-self.robot_position[1]) <= 2.0
                 and math.hypot(points[-1][0]-endpoint[0], points[-1][1]-endpoint[1]) <= 1.5)
        if valid:
            safe = not self.path_repeats_failed_pocket(points)
            self.resume_reason_pub.publish(String(data="accepted route endpoint=%s original=%s" % (endpoint, resume)
                if safe else "rejected: actual route crosses failed pocket"))
            return safe
        reason = "waiting: stale/empty path or mismatched start/endpoint; expected=%s original=%s" % (endpoint, resume)
        if len(points) >= 2 and math.hypot(points[-1][0]-endpoint[0], points[-1][1]-endpoint[1]) > 1.5:
            reason = "waiting: endpoint mismatch actual=%s expected=%s original=%s" % (points[-1], endpoint, resume)
        if getattr(self, "last_resume_reason", None) != reason:
            self.resume_reason_pub.publish(String(data=reason))
            self.last_resume_reason = reason
        return False if (now-probe[1]).to_sec() >= 4.0 else None

    def follow_test_intermediate(self, now):
        goal = getattr(self, "test_intermediate", None)
        if goal is None:
            return False
        distance = math.hypot(goal[0] - self.robot_position[0], goal[1] - self.robot_position[1])
        if distance <= 1.5:
            resume = getattr(self, "recovery_resume_goal", None)
            if self.test_goal is not None:
                resume = (self.test_goal.point.x, self.test_goal.point.y)
            validation = True
            if resume is not None and self.active_failed_pockets():
                validation = self.validate_resume_path(resume, now)
                if validation is None:
                    self.stop_publisher.publish(Int8(data=2))
                    self.turn_rate_publisher.publish(Float32(data=0.0))
                    self.publish_state("checking actual FAR route before resuming goal", "REPLANNING")
                    return True
            if resume is not None and (not validation or self.route_crosses_failed_pocket(*resume)):
                # Reaching a retreat point does not establish a bypass. Seek
                # lateral graph candidates, never immediately retry the pocket.
                dx = resume[0] - self.robot_position[0]
                dy = resume[1] - self.robot_position[1]
                norm = max(1.e-6, math.hypot(dx, dy))
                self.remember_failed_goal(goal)
                candidate = self.select_topological_goal(dx / norm, dy / norm, side=1)
                if candidate is None:
                    candidate = self.select_topological_goal(dx / norm, dy / norm, side=-1)
                if candidate is not None:
                    self.resume_path_probe = None
                    self.recovery_candidate_wait_started = None
                    self.test_intermediate = candidate
                    self.test_intermediate_started = now
                    self.test_intermediate_progress = now
                    self.test_intermediate_best = float("inf")
                    self.test_last_publish = rospy.Time(0)
                    self.publish_call_goal(candidate[0], candidate[1], now)
                    self.publish_state("retreat complete; pursuing lateral bypass candidate", "RECOVERY_DETOUR")
                    return True
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.publish_state("no lateral bypass candidate; refusing repeated failed approach", "REPLANNING")
                if getattr(self, "recovery_candidate_wait_started", None) is None:
                    self.recovery_candidate_wait_started = now
                if (now - self.recovery_candidate_wait_started).to_sec() >= 12.0:
                    self.active = False
                    self.publish_state("recovery failed: no lateral bypass candidate", "RECOVERY_FAILED")
                return True
            self.test_intermediate = None
            self.resume_path_probe = None
            self.test_last_publish = rospy.Time(0)
            return False
        if distance < self.test_intermediate_best - 0.25:
            self.test_intermediate_best = distance
            self.test_intermediate_progress = now
        if (now - self.test_intermediate_progress).to_sec() >= 12.0:
            # Do not silently fall back to the final goal after a failed detour.
            self.active = False
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("recovery failed: intermediate goal made no progress for 12 seconds", "RECOVERY_FAILED")
            return True
        self.stop_publisher.publish(Int8(data=0))
        self.turn_rate_publisher.publish(Float32(data=0.0))
        if (now - self.test_last_publish).to_sec() >= 1.0:
            self.publish_call_goal(goal[0], goal[1], now)
            self.test_last_publish = now
        self.publish_state("following recovery intermediate: %.2f m" % distance, "TEST_DETOUR")
        return True

    def timer_callback(self, _event):
        with self.lock:
            self.publish_visualization()
            if (
                self.state == "COMPLETE"
                and self.remaining_victims is not None
                and self.remaining_victims > 0
            ):
                self.active = True
                self.completion_candidate_since = None
                self.publish_state(
                    "resuming: Unity still reports %d victim(s)"
                    % self.remaining_victims,
                    "LISTENING",
                )
            if not self.active:
                self.near_target_publisher.publish(Bool(data=False))
                return

            if self.test_goal is not None:
                self.run_navigation_test(rospy.Time.now())
                return

            if (
                self.mission_max_duration > 0.0
                and self.mission_started_at is not None
                and (rospy.Time.now() - self.mission_started_at).to_sec()
                >= self.mission_max_duration
            ):
                self.active = False
                self.current_target_id = None
                self.near_target_publisher.publish(Bool(data=False))
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.publish_state(
                    "mission timeout after {:.0f} seconds".format(
                        self.mission_max_duration
                    ),
                    "TIMEOUT",
                )
                rospy.logwarn("Victim mission timed out; stopping vehicle")
                return

            # Unity removes rescued victim objects from the scene, so a
            # transient remaining-count of zero must not by itself complete
            # the mission.  Completion requires the rescued count to reach
            # the fixed episode total; this prevents 1/2 from becoming
            # COMPLETE after the first victim is destroyed.
            completion_ready = (
                self.expected_victim_count > 0
                and self.total_rescued_count() >= self.expected_victim_count
                and self.remaining_victims == 0
            )
            if completion_ready:
                if self.completion_candidate_since is None:
                    self.completion_candidate_since = rospy.Time.now()
                    return
                if (rospy.Time.now() - self.completion_candidate_since).to_sec() < 1.5:
                    return
            else:
                self.completion_candidate_since = None

            if completion_ready:
                self.active = False
                self.current_target_id = None
                self.near_target_publisher.publish(Bool(data=False))
                self.last_goal_position = None
                self.arrival_started_at = None
                self.call_contact_started_at = None
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.publish_state(
                    "mission complete: all simulated victims rescued",
                    "COMPLETE",
                )
                rospy.logwarn("Victim mission complete; stopping vehicle")
                return

            # Use Unity's planar rescue ring as the authoritative final-metre
            # decision. Perception remains responsible for finding and
            # approaching the victim, but residual map/depth error cannot keep
            # the vehicle rotating after it is physically inside the ring.
            if (
                self.in_rescue_range
                and self.rescue_zone_armed
                and self.remaining_victims is not None
                and self.remaining_victims > 0
            ):
                self.near_target_publisher.publish(Bool(data=True))
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.publish_state("waiting for Unity rescue confirmation", "RESCUING")
                return  # Unity alone performs the hold and commits the result.
                if self.contact_rescue_pending_remaining is not None:
                    self.publish_state(
                        "physical rescue registered; awaiting Unity acknowledgement",
                        "VICTIM_RESCUED",
                    )
                    return
                if self.call_contact_started_at is None:
                    self.call_contact_started_at = rospy.Time.now()
                held = (rospy.Time.now() - self.call_contact_started_at).to_sec()
                if held >= self.rescue_hold:
                    self.call_rescued_count = min(
                        self.expected_victim_count,
                        self.call_rescued_count + 1,
                    )
                    self.rescue_zone_armed = False
                    self.contact_rescue_pending_remaining = self.remaining_victims
                    self.call_contact_started_at = None
                    # Close the perception track that led us into this
                    # physical rescue zone. Otherwise the same stale track can
                    # immediately trigger the map-distance rescue path again.
                    physical_track = next(
                        (
                            track for track in self.tracks
                            if track.track_id == self.current_target_id
                            and not track.rescued
                        ),
                        None,
                    )
                    if physical_track is not None:
                        physical_track.rescued = True
                        self.remember_rescued_position(
                            physical_track.x,
                            physical_track.y,
                            physical_track.z,
                        )
                    self.current_target_id = None
                    self.last_goal_position = None
                    self.publish_state(
                        "victim rescued by Unity proximity; selecting next",
                        "VICTIM_RESCUED",
                    )
                else:
                    self.publish_state(
                        "confirming Unity rescue proximity: {:.1f}/{:.1f} s".format(
                            held, self.rescue_hold
                        ),
                        "RESCUING",
                    )
                return

            now = rospy.Time.now()
            if self.handle_wall_blocked_exploration(now):
                return
            if self.handle_fire_blocked_exploration(now):
                return
            if self.handle_replan_request(now):
                return
            if self.local_recovery_active:
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.publish_state("retaining target while local safety recovery executes", "LOCAL_RECOVERY")
                return
            if self.follow_test_intermediate(now):
                return
            if now < getattr(self, "recovery_exploration_until", rospy.Time(0)):
                if self.publish_autonomous_exploration_goal():
                    self.publish_state(self.call_guidance_status, "AUTONOMOUS_EXPLORATION")
                    return
            target = self.nearest_available_track()
            if target is None:
                self.near_target_publisher.publish(Bool(data=False))
                self.current_target_id = None
                self.arrival_started_at = None
                if self.publish_visual_guidance_goal():
                    self.publish_state(
                        self.call_guidance_status, "VISUAL_GUIDANCE"
                    )
                elif self.publish_call_guidance_goal():
                    self.publish_state(
                        self.call_guidance_status, "CALL_GUIDANCE"
                    )
                elif self.publish_autonomous_exploration_goal():
                    self.publish_state(
                        self.call_guidance_status, "AUTONOMOUS_EXPLORATION"
                    )
                else:
                    self.stop_publisher.publish(Int8(data=2))
                    self.turn_rate_publisher.publish(Float32(data=0.0))
                    self.publish_state(
                        "active: waiting for an unrescued victim", "LISTENING"
                    )
                return

            # Rescue proximity must be evaluated before standoff retry and
            # local-recovery logic. Previously a vehicle already beside a
            # victim could rotate the standoff candidate first, which made the
            # recovery controller enter wall-follow/trace-back during the
            # rescue hold.
            distance = math.hypot(
                target.x - self.robot_position[0],
                target.y - self.robot_position[1],
            )
            # A confirmed, precisely localized victim owns navigation until
            # rescue or bounded standoff failure. Suppress heuristic recovery
            # throughout this approach; Unity collision events remain active.
            near_target = distance <= self.precise_target_range
            self.near_target_publisher.publish(Bool(data=near_target))
            if (
                False  # Distance estimates never authorize a scene rescue.
                and distance <= self.rescue_distance
            ):
                self.current_target_id = target.track_id
                self.turn_rate_publisher.publish(Float32(data=0.0))
                self.stop_publisher.publish(Int8(data=2))
                if self.arrival_started_at is None:
                    self.arrival_started_at = rospy.Time.now()
                    self.publish_state(
                        "holding position near victim {}".format(target.track_id),
                        "RESCUING",
                    )
                elif ((rospy.Time.now() - self.arrival_started_at).to_sec()
                      >= self.rescue_hold):
                    target.rescued = True
                    # Publish the rescue event immediately. Previously the
                    # public count was derived only from Unity's remaining
                    # count, while Unity waited for this public count before
                    # silencing the victim: both sides waited forever.
                    self.call_rescued_count = min(
                        self.expected_victim_count,
                        self.call_rescued_count + 1,
                    )
                    self.rescue_zone_armed = False
                    self.contact_rescue_pending_remaining = self.remaining_victims
                    self.remember_rescued_position(target.x, target.y, target.z)
                    rospy.logwarn("Victim %d marked rescued", target.track_id)
                    self.current_target_id = None
                    self.last_goal_position = None
                    self.arrival_started_at = None
                    self.near_target_publisher.publish(Bool(data=False))
                    self.stop_publisher.publish(Int8(data=0))
                    self.publish_state(
                        "victim {} rescued; selecting next".format(target.track_id),
                        "VICTIM_RESCUED",
                    )
                return
            self.arrival_started_at = None

            self.detour_goal = None
            self.detour_locked_side = None
            self.detour_side_lock_until = rospy.Time(0)
            self.best_call_distance = None
            self.call_progress_time = rospy.Time.now()

            target_changed = target.track_id != self.current_target_id
            if target_changed:
                self.standoff_candidate_index = 0
                self.standoff_attempted_indices = {0}
                self.standoff_zero_since = None
                self.standoff_base_angle = math.atan2(
                    self.robot_position[1] - target.y,
                    self.robot_position[0] - target.x,
                )
                self.standoff_best_distance = math.hypot(
                    target.x - self.robot_position[0],
                    target.y - self.robot_position[1],
                )
                self.standoff_progress_time = rospy.Time.now()
                if not self.standoff_candidate_is_clear(target, 0):
                    if not self.advance_standoff_candidate(
                            target, "initial standoff blocked by obstacle"):
                        self.publish_state(
                            "victim {} deferred; exploring alternate area".format(
                                target.track_id
                            ),
                            "TARGET_DEFERRED",
                        )
                        return
            else:
                target_distance = math.hypot(
                    target.x - self.robot_position[0],
                    target.y - self.robot_position[1],
                )
                if (self.standoff_best_distance is None
                        or target_distance <= self.standoff_best_distance
                        - self.standoff_progress_distance):
                    self.standoff_best_distance = target_distance
                    self.standoff_progress_time = rospy.Time.now()
                stalled = (
                    (rospy.Time.now() - self.standoff_progress_time).to_sec()
                    >= self.standoff_retry_timeout
                )
                fresh_raw_command = (
                    (rospy.Time.now() - self.raw_command_received_at).to_sec()
                    <= 1.0
                )
                raw_is_zero = (
                    fresh_raw_command
                    and abs(self.raw_linear_speed) < 0.05
                    and abs(self.raw_angular_speed) < 0.08
                    and not self.local_recovery_active
                )
                if raw_is_zero:
                    if self.standoff_zero_since is None:
                        self.standoff_zero_since = rospy.Time.now()
                else:
                    self.standoff_zero_since = None
                zero_stalled = (
                    self.standoff_zero_since is not None
                    and (rospy.Time.now() - self.standoff_zero_since).to_sec()
                    >= self.standoff_zero_command_timeout
                )
                if stalled or zero_stalled:
                    reason = (
                        "local planner produced zero velocity"
                        if zero_stalled else "approach made no progress"
                    )
                    if not self.advance_standoff_candidate(target, reason):
                        self.publish_state(
                            "victim {} deferred; resuming exploration".format(
                                target.track_id
                            ),
                            "TARGET_DEFERRED",
                        )
                        return
            goal_moved = (
                self.last_goal_position is None
                or math.hypot(
                    target.x - self.last_goal_position[0],
                    target.y - self.last_goal_position[1],
                ) >= self.republish_distance
            )
            goal_expired = (
                rospy.Time.now() - self.last_goal_time
            ).to_sec() >= self.goal_republish_interval
            if target_changed or goal_moved or goal_expired:
                self.current_target_id = target.track_id
                self.publish_goal(target)

            self.publish_state(
                "navigating to victim {}: {:.2f} m remaining".format(
                    target.track_id, distance
                ),
                "NAVIGATING",
            )

    def publish_visual_guidance_goal(self):
        now = rospy.Time.now()
        if now < getattr(self, "_visual_retry_after", rospy.Time(0)):
            return False
        if not self.visual_observation_available(now):
            self.clear_visual_guidance()
            return False
        if self.local_recovery_active:
            self.call_guidance_status = "visual target retained during safe recovery"
            return True
        if not self.visual_guidance_active:
            self.last_call_goal_position = None
            self.last_call_goal_time = rospy.Time(0)
            self.detour_goal = None
            self.visual_guidance_active = True
            self._visual_lease_start = now
            self._visual_motion_time = now
            self._visual_motion_anchor = self.robot_position[:2]
        if math.hypot(self.robot_position[0]-self._visual_motion_anchor[0],
                      self.robot_position[1]-self._visual_motion_anchor[1]) >= 1.0:
            self._visual_motion_anchor = self.robot_position[:2]
            self._visual_motion_time = now
        if ((now-self._visual_motion_time).to_sec() >= 15.0 or
                (now-self._visual_lease_start).to_sec() >= 60.0):
            rospy.logwarn("Unlocalized visual approach stalled/exhausted; yield to exploration for 30 s")
            self._visual_retry_after = now + rospy.Duration(30.0)
            self.clear_visual_guidance()
            self.last_call_goal_position = None
            self.last_call_goal_time = rospy.Time(0)
            return False
        # A stable map-frame goal, not an independent turn command. FAR and
        # the local planner can turn AND avoid obstacles on the approach.
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))
        replace = self.visual_goal_position is None
        if not replace:
            dx = self.visual_goal_position[0] - self.robot_position[0]
            dy = self.visual_goal_position[1] - self.robot_position[1]
            replace = (math.hypot(dx, dy) <= self.visual_goal_arrival_distance or
                abs(self.angle_difference(math.atan2(dy, dx), self.visual_world_angle))
                > self.visual_direction_tolerance)
        if replace and (self.visual_goal_position is None or
                (now - self.last_visual_goal_time).to_sec() >= self.visual_goal_interval):
            angle = self.visual_world_angle
            x = self.robot_position[0] + self.visual_goal_distance * math.cos(angle)
            y = self.robot_position[1] + self.visual_goal_distance * math.sin(angle)
            self.visual_goal_position = (
                min(self.map_max_x, max(self.map_min_x, x)),
                min(self.map_max_y, max(self.map_min_y, y)))
            self.publish_call_goal(*self.visual_goal_position, now)
            self.last_visual_goal_time = now
        elif (now - self.last_visual_goal_time).to_sec() >= self.visual_goal_keepalive:
            self.publish_call_goal(*self.visual_goal_position, now)
            self.last_visual_goal_time = now
        self.call_guidance_status = "approaching visually detected victim for LiDAR localization"
        return True

    def publish_call_guidance_goal(self):
        if not getattr(self, 'enable_simulated_call_guidance', True):
            return False
        now = rospy.Time.now()
        memory = getattr(self, '_acoustic_memory', None)
        memory_valid = (getattr(self, 'enable_simulated_call_guidance', True)
                        and memory is not None and 0 <= now.to_sec()-memory[1] <= 60.0)
        live_call = (self.call_detected and self.call_bearing is not None
                     and (now-self.call_received_at).to_sec() <= self.call_timeout)
        if (
            not (live_call or memory_valid)
            or self.robot_position is None
            or not hasattr(self, "robot_yaw")
        ):
            return False

        if self.contact_rescue_pending_remaining is not None:
            # The previous victim has been physically removed.  Do not let a
            # stale bearing or committed FAR waypoint reacquire it while the
            # next Unity call is being published; this path must also work
            # when call-strength estimation is temporarily unavailable.
            self.detour_goal = None
            self.clear_committed_target()
            self.call_detected = False
            self.call_bearing = None
            self.call_strength = None
            self.call_precise_position = None
            self.call_precise_received_at = rospy.Time(0)
            self.stop_publisher.publish(Int8(data=0))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.call_guidance_status = (
                "contact rescue registered; waiting for next victim call"
            )
            return True

        forward, left = self.call_bearing or (1.0, 0.0)
        goal_distance = self.call_goal_distance

        # Unity publishes a distance-dependent noisy acoustic estimate.
        # Confirmed visual tracks and visual guidance take priority in tick().
        # FAR owns obstacle avoidance; physical rescue proximity is separate
        # ground-truth simulation instrumentation, not a localization result.
        now = rospy.Time.now()
        if now.to_sec() < getattr(self, '_acoustic_scan_yield_until', 0.0):
            return self.publish_autonomous_exploration_goal()
        precise_call_available = (
            self.call_precise_position is not None
            and (now - self.call_precise_received_at).to_sec()
                <= max(self.call_timeout, 1.0)
        )
        estimate = self.call_precise_position if precise_call_available else (memory[0] if memory_valid else None)
        precise_call_available = estimate is not None
        if precise_call_available:
            estimate_x = min(self.map_max_x, max(
                self.map_min_x, estimate[0]))
            estimate_y = min(self.map_max_y, max(
                self.map_min_y, estimate[1]))
            # Acoustic positions deliberately contain distance-dependent
            # error. Commit a short carrot rather than continuously sending
            # FAR a new supposed final destination; each tiny revision made
            # the local planner stop and pivot in open space.
            dx = estimate_x - self.robot_position[0]
            dy = estimate_y - self.robot_position[1]
            estimate_distance = math.hypot(dx, dy)
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.stop_publisher.publish(Int8(data=0))
            if self.maintain_acoustic_goal(
                    now, dx / max(estimate_distance, 1e-6),
                    dy / max(estimate_distance, 1e-6)):
                return True
            # An acoustic estimate can lie inside the person's collider (or
            # noisy occupied space). Request a sensing standoff, never require
            # a collision-free route all the way into the estimate itself.
            travel = min(self.call_goal_distance, max(0.0, estimate_distance - 2.7))
            if travel < 1.0:
                started = getattr(self, '_acoustic_near_scan_started', None)
                if started is None:
                    self._acoustic_near_scan_started = now.to_sec()
                elif now.to_sec() - started >= 12.0:
                    self._acoustic_near_scan_started = None
                    self._acoustic_scan_yield_until = now.to_sec() + 25.0
                    self.call_approach_goal_position = None
                    return self.publish_autonomous_exploration_goal()
                self.call_guidance_status = 'near acoustic estimate; scanning for visual confirmation'
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.25))
                return True
            self._acoustic_near_scan_started = None
            if estimate_distance > 1e-6:
                goal_x = self.robot_position[0] + travel * dx / estimate_distance
                goal_y = self.robot_position[1] + travel * dy / estimate_distance
            else:
                goal_x, goal_y = self.robot_position[0], self.robot_position[1]
            acoustic_goal = (goal_x, goal_y)
            changed = (
                self.call_approach_goal_position is None
                or math.hypot(
                    acoustic_goal[0] - self.call_approach_goal_position[0],
                    acoustic_goal[1] - self.call_approach_goal_position[1],
                ) >= self.acoustic_goal_update_distance
            )
            elapsed = (now - self.call_approach_goal_time).to_sec()
            self.detour_goal = None
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.stop_publisher.publish(Int8(data=0))
            if (self.call_approach_goal_position is None or
                    (changed and elapsed >= self.acoustic_goal_update_interval)):
                self.call_approach_goal_position = acoustic_goal
                self.publish_approach_goal(goal_x, goal_y, now)
                rospy.loginfo(
                    "CALL_GUIDANCE acoustic stage goal (%.2f, %.2f), estimate %.2f m",
                    goal_x, goal_y,
                    estimate_distance,
                )
            if self.call_approach_goal_position is None:
                replacement = self.select_topological_goal(
                    dx / max(estimate_distance, 1e-6),
                    dy / max(estimate_distance, 1e-6), unconstrained=True,
                    target_hint=(estimate_x, estimate_y))
                if replacement is not None:
                    self.call_approach_goal_position = replacement
                    self._acoustic_progress = None
                    self.publish_approach_goal(*replacement, now)
                    self.call_guidance_status = 'following reachable acoustic detour'
                    return True
                # Avoid recomputing disconnected noisy carrots every timer tick.
                self._acoustic_scan_yield_until = now.to_sec() + 10.0
                return self.publish_autonomous_exploration_goal()
            remaining = math.hypot(
                self.call_approach_goal_position[0] - self.robot_position[0],
                self.call_approach_goal_position[1] - self.robot_position[1],
            )
            self.call_guidance_status = (
                "following stable acoustic stage goal: {:.2f} m".format(
                    remaining
                )
            )
            return True

        if self.call_strength is not None:
            estimated_distance = self.call_hearing_distance * (
                1.0 - self.call_strength
            )
            now = rospy.Time.now()
            if self.detour_goal is None and (
                self.best_call_distance is None
                or estimated_distance
                <= self.best_call_distance - self.stuck_progress_distance
            ):
                self.best_call_distance = estimated_distance
                self.call_progress_time = now
            if (
                False  # Distance estimates never authorize a scene rescue.
                and estimated_distance <= self.contact_rescue_distance
            ):
                self.detour_goal = None
                self.stop_publisher.publish(Int8(data=2))
                self.turn_rate_publisher.publish(Float32(data=0.0))
                if not self.rescue_zone_armed:
                    self.call_guidance_status = (
                        "waiting to leave completed victim rescue zone"
                    )
                    return True
                if self.call_contact_started_at is None:
                    self.call_contact_started_at = rospy.Time.now()
                held = (rospy.Time.now() - self.call_contact_started_at).to_sec()
                if held >= self.contact_rescue_hold:
                    self.call_rescued_count = min(
                        self.expected_victim_count,
                        self.call_rescued_count + 1,
                    )
                    self.rescue_zone_armed = False
                    self.contact_rescue_pending_remaining = self.remaining_victims
                    self.call_contact_started_at = None
                    self.call_approach_goal_position = None
                    self.call_approach_goal_time = rospy.Time(0)
                    self.call_guidance_status = (
                        "victim rescued by physical contact; selecting next"
                    )
                    rospy.logwarn(
                        "Victim rescued by contact at estimated distance %.2f m",
                        estimated_distance,
                    )
                else:
                    self.call_guidance_status = (
                        "confirming physical contact with victim: {:.1f} s".format(
                            held
                        )
                    )
                return True
            self.call_contact_started_at = None

            direction_x = (
                math.cos(self.robot_yaw) * forward
                - math.sin(self.robot_yaw) * left
            )
            direction_y = (
                math.sin(self.robot_yaw) * forward
                + math.cos(self.robot_yaw) * left
            )
            if self.publish_wall_bypass_goal(
                direction_x, direction_y, estimated_distance
            ):
                return True

            turn_timed_out = False
            heading_error = 0.0
            if estimated_distance <= self.call_standoff_distance + 0.25:
                heading_error = math.atan2(left, forward)
                if abs(heading_error) > self.call_heading_tolerance:
                    now = rospy.Time.now()
                    if self.call_turn_started_at is None:
                        self.call_turn_started_at = now
                    turn_elapsed = (now - self.call_turn_started_at).to_sec()
                    if turn_elapsed < self.call_turn_hold_seconds:
                        self.stop_publisher.publish(Int8(data=2))
                        turn_rate = max(
                            -self.call_max_turn_rate,
                            min(
                                self.call_max_turn_rate,
                                self.call_turn_gain * heading_error,
                            ),
                        )
                        self.turn_rate_publisher.publish(Float32(data=turn_rate))
                        self.call_guidance_status = (
                            "turning toward nearby victim call: {:.1f} deg "
                            "({:.1f}s)".format(
                                math.degrees(heading_error), turn_elapsed
                            )
                        )
                        return True
                    # Bearing did not converge. Release the direct turn and
                    # send a finite forward goal so the planners can move
                    # around the reflecting wall instead of spinning forever.
                    self.call_turn_started_at = None
                    turn_timed_out = True
                    self.call_guidance_status = (
                        "acoustic bearing stalled; handing off to planner"
                    )
                else:
                    self.call_turn_started_at = None
                    self.turn_rate_publisher.publish(Float32(data=0.0))
                    self.call_guidance_status = (
                        "routing around occlusion near victim call"
                    )
            goal_distance = min(
                self.call_goal_distance,
                max(
                    self.call_min_goal_distance,
                    estimated_distance - self.call_standoff_distance,
                ),
            )
            if turn_timed_out:
                goal_distance = max(goal_distance, self.call_goal_distance)
            self.call_guidance_status = (
                "following victim call: estimated {:.2f} m".format(
                    estimated_distance
                )
            )
        else:
            self.call_guidance_status = "following simulated victim call"

        # A valid call owns navigation now; release any previous safety stop.
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))
        direction_x = math.cos(self.robot_yaw) * forward - math.sin(self.robot_yaw) * left
        direction_y = math.sin(self.robot_yaw) * forward + math.cos(self.robot_yaw) * left
        now = rospy.Time.now()
        elapsed = (now - self.call_approach_goal_time).to_sec()

        # Commit to one exploration waypoint until it is reached. Replacing a
        # moving carrot every few seconds caused FAR to repeatedly clear and
        # rebuild its local graph, wasting time and producing RViz flicker.
        if self.call_approach_goal_position is not None:
            committed_remaining = math.hypot(
                self.call_approach_goal_position[0] - self.robot_position[0],
                self.call_approach_goal_position[1] - self.robot_position[1],
            )
            progress_age = self.exploration_progress_age(
                self.call_approach_goal_position, committed_remaining, now)
            coverage_stalled = (
                not self.local_recovery_active
                and progress_age >= self.coverage_stall_timeout
            )
            if coverage_stalled:
                rospy.logwarn(
                    "Exploration made no goal-distance or coverage progress for %.1f s; switching region",
                    progress_age,
                )
                self.remember_failed_goal(self.call_approach_goal_position)
                self.call_approach_goal_position = None
                self.call_approach_goal_time = rospy.Time(0)
                self.last_coverage_gain_time = now
            elif committed_remaining > self.exploration_goal_arrival_distance:
                self.call_guidance_status = (
                    "following committed call approach: {:.2f} m".format(
                        committed_remaining
                    )
                )
                if elapsed >= self.call_goal_interval:
                    self.publish_approach_goal(
                        self.call_approach_goal_position[0],
                        self.call_approach_goal_position[1],
                        now,
                    )
                return True
            else:
                self.call_approach_goal_position = None
                self.call_approach_goal_time = rospy.Time(0)
            self.best_call_distance = (
                estimated_distance if self.call_strength is not None else None
            )
            self.call_progress_time = now

        topology_goal = self.select_topological_goal(direction_x, direction_y)
        if topology_goal is not None:
            goal_x, goal_y = topology_goal
            self.call_guidance_status = "following unvisited FAR topology node"
        else:
            goal_x = self.robot_position[0] + goal_distance * direction_x
            goal_y = self.robot_position[1] + goal_distance * direction_y
            goal_x = min(self.map_max_x, max(self.map_min_x, goal_x))
            goal_y = min(self.map_max_y, max(self.map_min_y, goal_y))

        # Re-publish periodically even when the temporary target is unchanged.
        # FAR's goal topic is not latched, so a goal sent while its graph is
        # initializing (or while it is replanning) would otherwise be lost.
        if elapsed >= self.call_goal_interval:
            self.call_approach_goal_position = (goal_x, goal_y)
            self.publish_approach_goal(goal_x, goal_y, now)
            rospy.loginfo(
                "Following victim call toward stable goal (%.2f, %.2f)",
                goal_x,
                goal_y,
            )
        return True

    def exploration_progress_age(self, goal, remaining, now):
        """Only new coverage or a new distance minimum counts as progress.

        Displacement alone rewards oscillation. Keep a cumulative best distance
        for each committed goal, so returning to an already reached distance
        cannot renew the deadline. Coverage still permits genuine detours.
        """
        anchor = getattr(self, '_exploration_motion_anchor', None)
        if (anchor is None or now < anchor[1] or
                math.dist(self.robot_position[:2], anchor[0]) >= 0.5 or
                self.local_recovery_active):
            anchor = (tuple(self.robot_position[:2]), now)
            self._exploration_motion_anchor = anchor
        stationary_age = (now - anchor[1]).to_sec()
        key = tuple(goal[:2])
        route = getattr(self, '_observed_route', [])
        if getattr(self, '_observed_route_goal', None) == key and route:
            # A detour can increase Euclidean distance to the final goal.
            # Judge progress toward the current executable leg instead.
            remaining = math.dist(self.robot_position[:2], route[0])
            key = (key, tuple(route[0]))
        previous = getattr(self, "_exploration_progress", None)
        if previous is None or previous[0] != key or now < previous[2]:
            self._exploration_progress = (key, remaining, now)
            return max(0.0, stationary_age)
        _, best, improved_at = previous
        if remaining <= best - 0.5:
            best, improved_at = remaining, now
        if self.local_recovery_active:
            improved_at = now
        self._exploration_progress = (key, best, improved_at)
        return max(0.0, stationary_age, min(
            (now - improved_at).to_sec(),
            (now - self.last_coverage_gain_time).to_sec()))

    def publish_autonomous_exploration_goal(self):
        """Explore high-information FAR nodes when no victim call is audible."""
        if self.robot_position is None or not hasattr(self, "robot_yaw"):
            return False

        now = rospy.Time.now()
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))

        if self.last_call_goal_position is not None:
            remaining = math.hypot(
                self.last_call_goal_position[0] - self.robot_position[0],
                self.last_call_goal_position[1] - self.robot_position[1],
            )
            progress_age = self.exploration_progress_age(
                self.last_call_goal_position, remaining, now)
            coverage_stalled = (
                not self.local_recovery_active
                and progress_age >= self.coverage_stall_timeout
            )
            if coverage_stalled:
                self.remember_failed_goal(self.last_call_goal_position)
                self._exploration_motion_anchor = (tuple(self.robot_position[:2]), now)
                rospy.logwarn(
                    "Autonomous exploration stalled; blacklisting (%.2f, %.2f)",
                    self.last_call_goal_position[0],
                    self.last_call_goal_position[1],
                )
                self.last_call_goal_position = None
                self.last_call_goal_time = rospy.Time(0)
                self.last_coverage_gain_time = now
            elif remaining > self.exploration_goal_arrival_distance:
                if (
                    now - self.last_call_goal_time
                ).to_sec() >= self.call_goal_interval:
                    self.publish_exploration_leg(
                        self.last_call_goal_position[0],
                        self.last_call_goal_position[1],
                        now,
                    )
                self.call_guidance_status = (
                    "exploration target distance: {:.2f} m".format(
                        remaining
                    )
                )
                return True
            else:
                self.last_call_goal_position = None
                self.last_call_goal_time = rospy.Time(0)

        heading_x = math.cos(self.robot_yaw)
        heading_y = math.sin(self.robot_yaw)
        goal = self.select_topological_goal(
            heading_x, heading_y, unconstrained=True
        )
        if goal is None:
            rospy.logwarn_throttle(10.0, "No exploration candidate: %s", getattr(
                self, "recovery_selection_diagnostic", "no graph/coverage"))
            self.call_guidance_status = (
                "scanning for a safe FAR node; failed corridors remain blocked"
            )
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.25))
            return True

        self.last_call_goal_position = goal
        self.publish_exploration_leg(goal[0], goal[1], now)
        self.call_guidance_status = "autonomously exploring highest-coverage FAR node"
        rospy.loginfo(
            "No victim call; autonomous exploration goal (%.2f, %.2f)",
            goal[0], goal[1]
        )
        return True

    def observed_route_points(self):
        radius = self.recovery_goal_clearance - 1.5 + 0.25
        return [((cx+.5)*self.coverage_cell_size, (cy+.5)*self.coverage_cell_size)
                for cx,cy in self.coverage_cells
                if self.endpoint_clear((cx+.5)*self.coverage_cell_size,
                                       (cy+.5)*self.coverage_cell_size, radius)]

    def publish_exploration_leg(self, goal_x, goal_y, now):
        goal=(goal_x,goal_y)
        route=getattr(self,'_observed_route',[])
        if getattr(self,'_observed_route_goal',None)!=goal:
            route=[]
        while route and math.dist(self.robot_position[:2],route[0])<=1.5:
            route.pop(0)
        # FAR may stop at a clearance-adjusted point short of our original
        # leg. Advance only when the next swept corridor is directly safe;
        # exact arrival at the old point is then unnecessary.
        # A* limits edges to 12 m, but FAR can adjust the prior endpoint.
        # Allow the span of two edges; requiring <=12 m here can strand the
        # robot just short of a leg forever. Swept clearance remains mandatory.
        while (len(route)>1 and math.dist(self.robot_position[:2],route[1])<=24.0
               and self.observed_segment_clear(self.robot_position[:2],route[1])):
            rospy.loginfo('Advancing observed leg %s -> %s from %s', route[0], route[1], self.robot_position[:2])
            route.pop(0)
        if not route or not self.observed_segment_clear(self.robot_position[:2],route[0]):
            points=self.observed_route_points()
            route=observed_route(self.robot_position[:2],goal,points,self.observed_segment_clear)
            self._observed_route_goal=goal
            rospy.loginfo('Observed-space handoff: %d legs toward (%.2f, %.2f)',len(route),goal_x,goal_y)
        self._observed_route=route
        if not route:
            # A disconnected observation graph is not evidence that an entire
            # physical region is blocked. Retry briefly, without poisoning the
            # long-lived failure memory around doors and transit corridors.
            retry = getattr(self, '_route_retry_after', {})
            self._route_retry_after = {p:t for p,t in retry.items() if t > now}
            self._route_retry_after[goal] = now + rospy.Duration(20.0)
            self.last_call_goal_position = None
            self.call_approach_goal_position = None
            self.last_call_goal_time = now
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.25))
            rospy.logwarn_throttle(5.0, 'No observed route: withholding unreachable goal %s', goal)
            return False
        self.publish_call_goal(*route[0],now)
        return True

    def publish_call_goal(self, goal_x, goal_y, now):
        self.last_published_navigation_goal = (goal_x, goal_y)
        goal = PointStamped()
        goal.header.stamp = now
        goal.header.frame_id = self.world_frame
        goal.point.x = goal_x
        goal.point.y = goal_y
        goal.point.z = self.robot_position[2] - self.vehicle_height
        self.goal_publisher.publish(goal)
        self.target_publisher.publish(goal)
        self.last_call_goal_time = now

    def publish_approach_goal(self, goal_x, goal_y, now):
        exploration_time = self.last_call_goal_time
        self.publish_exploration_leg(goal_x, goal_y, now)
        self.last_call_goal_time = exploration_time
        self.call_approach_goal_time = now

    def publish_wall_bypass_goal(
        self, direction_x, direction_y, estimated_distance
    ):
        """Escape a repeated straight-line goal by exploring along the wall."""
        if not self.enable_wall_bypass:
            self.detour_goal = None
            self.detour_locked_side = None
            return False
        now = rospy.Time.now()

        # The local controller has fresher obstacle geometry and preserves the
        # current FAR waypoint. Do not create a competing acoustic detour while
        # it is already backing, pivoting or following a wall.
        if self.local_recovery_active and self.detour_goal is None:
            self.call_progress_time = now
            self.progress_anchor_position = self.robot_position
            self.progress_anchor_time = now
            return False

        if self.progress_anchor_position is None:
            self.progress_anchor_position = self.robot_position
            self.progress_anchor_time = now

        if self.detour_goal is not None:
            remaining = math.hypot(
                self.detour_goal[0] - self.robot_position[0],
                self.detour_goal[1] - self.robot_position[1],
            )
            if (
                self.detour_best_remaining is None
                or remaining
                <= self.detour_best_remaining - self.stuck_progress_distance
            ):
                self.detour_best_remaining = remaining
                self.detour_progress_time = now

            # Keep following the selected side while it is making progress.
            # Switch sides only after the detour itself has genuinely stalled.
            expired = (
                now - self.detour_progress_time
            ).to_sec() >= self.detour_duration
            if remaining <= self.detour_arrival_distance:
                self.detour_goal = None
                self.detour_best_remaining = None
                self.progress_anchor_position = self.robot_position
                self.progress_anchor_time = now
                self.best_call_distance = estimated_distance
                self.call_progress_time = now
                self.last_call_goal_time = rospy.Time(0)
                return False
            if expired:
                # The selected side was also blocked. Try the opposite side now.
                self.remember_failed_goal(self.detour_goal)
                self.detour_goal = None
                self.detour_best_remaining = None
            else:
                self.publish_detour_point(now)
                return True

        # Moving sideways or oscillating near a wall is not real mission
        # progress.  Judge stagnation from improvement in acoustic range.
        stalled_for = (now - self.call_progress_time).to_sec()
        if stalled_for < self.stuck_timeout:
            return False

        side_lock_active = (
            self.detour_locked_side is not None
            and now < self.detour_side_lock_until
        )
        preferred_side = (
            self.detour_locked_side
            if side_lock_active
            else self.detour_side
        )
        candidates = []
        candidate_sides = (
            (preferred_side,)
            if side_lock_active
            else (preferred_side, -preferred_side)
        )
        for side in candidate_sides:
            topology_goal = self.select_topological_goal(
                direction_x, direction_y, side=side
            )
            if topology_goal is not None:
                raw_x, raw_y = topology_goal
            else:
                tangent_x = -direction_y * side
                tangent_y = direction_x * side
                raw_x = (
                    self.robot_position[0]
                    + self.detour_forward_bias * direction_x
                    + self.detour_distance * tangent_x
                )
                raw_y = (
                    self.robot_position[1]
                    + self.detour_forward_bias * direction_y
                    + self.detour_distance * tangent_y
                )
            bounded_x = min(self.map_max_x, max(self.map_min_x, raw_x))
            bounded_y = min(self.map_max_y, max(self.map_min_y, raw_y))
            boundary_error = math.hypot(raw_x - bounded_x, raw_y - bounded_y)
            travel = math.hypot(
                bounded_x - self.robot_position[0],
                bounded_y - self.robot_position[1],
            )
            side_priority = 0 if side == preferred_side else 1
            candidates.append(
                (
                    boundary_error,
                    side_priority,
                    -travel,
                    side,
                    bounded_x,
                    bounded_y,
                )
            )

        # Prefer a side whose full detour stays inside the map. At an outer
        # wall this automatically rejects the outward tangent and follows the
        # wall back into navigable space. The travel tie-break avoids a
        # clamped point directly beneath the robot.
        _, _, _, side, goal_x, goal_y = min(candidates)
        if not side_lock_active:
            # Once a side is selected, keep exploring different topology
            # vertices on that same side. A single failed vertex must not make
            # the robot reverse its heading and oscillate left/right.
            self.detour_locked_side = side
            self.detour_side_lock_until = now + rospy.Duration.from_sec(
                max(self.detour_duration, self.detour_side_lock_duration)
            )
            self.detour_side = -side
        self.detour_goal = (goal_x, goal_y)
        self.remember_failed_goal(self.last_call_goal_position)
        self.last_call_goal_position = None
        self.last_call_goal_time = rospy.Time(0)
        self.detour_started_at = now
        self.detour_last_publish = rospy.Time(0)
        self.detour_best_remaining = math.hypot(
            self.detour_goal[0] - self.robot_position[0],
            self.detour_goal[1] - self.robot_position[1],
        )
        self.detour_progress_time = now
        self.progress_anchor_position = self.robot_position
        self.progress_anchor_time = now
        self.call_guidance_status = (
            "exploring locked {} side to bypass wall".format(
                "left" if side > 0.0 else "right"
            )
        )
        rospy.logwarn(
            "Call guidance stalled; exploring %s toward (%.2f, %.2f)",
            "left" if side > 0.0 else "right",
            self.detour_goal[0],
            self.detour_goal[1],
        )
        self.publish_detour_point(now)
        return True

    def publish_detour_point(self, now):
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))
        if (now - self.detour_last_publish).to_sec() >= self.detour_republish_interval:
            goal = PointStamped()
            goal.header.stamp = now
            goal.header.frame_id = self.world_frame
            goal.point.x = self.detour_goal[0]
            goal.point.y = self.detour_goal[1]
            goal.point.z = self.robot_position[2] - self.vehicle_height
            self.goal_publisher.publish(goal)
            self.target_publisher.publish(goal)
            self.detour_last_publish = now

    def publish_goal(self, track):
        # A confirmed victim target owns navigation now.
        self.turn_rate_publisher.publish(Float32(data=0.0))
        self.stop_publisher.publish(Int8(data=0))
        goal = PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.world_frame
        goal_x = track.x
        goal_y = track.y
        if self.robot_position is not None and self.navigation_standoff_distance > 0.0:
            victim_distance = math.hypot(
                self.robot_position[0] - track.x,
                self.robot_position[1] - track.y,
            )
            if self.navigation_standoff_distance > 0.0:
                if self.standoff_base_angle is None:
                    self.standoff_base_angle = math.atan2(
                        self.robot_position[1] - track.y,
                        self.robot_position[0] - track.x,
                    )
                offset = self.standoff_angle_offsets[
                    self.standoff_candidate_index
                    % max(1, len(self.standoff_angle_offsets))
                ] if self.standoff_angle_offsets else 0.0
                approach_angle = self.standoff_base_angle + offset
                goal_x = track.x + self.navigation_standoff_distance * math.cos(
                    approach_angle
                )
                goal_y = track.y + self.navigation_standoff_distance * math.sin(
                    approach_angle
                )
        goal.point.x = goal_x
        goal.point.y = goal_y
        # LiDAR normally hits the victim's body, but FAR expects a traversable
        self.last_published_navigation_goal = (goal_x, goal_y)
        # ground-level goal. Preserve victim X/Y and derive Z from vehicle pose.
        goal.point.z = (
            self.robot_position[2] - self.vehicle_height
            if self.robot_position is not None
            else track.z
        )
        self.goal_publisher.publish(goal)
        self.target_publisher.publish(goal)
        self.last_goal_position = (track.x, track.y)
        self.last_goal_time = goal.header.stamp
        rospy.logwarn(
            "Dispatching nearest victim %d via standoff (%.2f, %.2f), victim=(%.2f, %.2f)",
            track.track_id,
            goal_x,
            goal_y,
            track.x,
            track.y,
        )

    def publish_visualization(self):
        confirmed = self.confirmed_tracks()
        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = self.world_frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for track in confirmed:
            pose = Pose()
            pose.position.x = track.x
            pose.position.y = track.y
            pose.position.z = track.z
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            sphere = Marker()
            sphere.header = poses.header
            sphere.ns = "victim_mission"
            sphere.id = track.track_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = track.x
            sphere.pose.position.y = track.y
            sphere.pose.position.z = track.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.7
            sphere.scale.y = 0.7
            sphere.scale.z = 0.7
            if track.rescued:
                sphere.color.r = 0.4
                sphere.color.g = 0.4
                sphere.color.b = 0.4
            elif track.track_id == self.current_target_id:
                sphere.color.g = 1.0
            else:
                sphere.color.r = 1.0
                sphere.color.g = 0.85
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            label = Marker()
            label.header = poses.header
            label.ns = "victim_mission_labels"
            label.id = track.track_id * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = track.x
            label.pose.position.y = track.y
            label.pose.position.z = track.z + 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.45
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            state = "RESCUED" if track.rescued else "TARGET" if track.track_id == self.current_target_id else "WAITING"
            label.text = "Victim {} {}".format(track.track_id, state)
            markers.markers.append(label)

        self.known_publisher.publish(poses)
        self.marker_publisher.publish(markers)
        self.rescued_count_publisher.publish(
            Int32(data=self.total_rescued_count())
        )

    def total_rescued_count(self):
        if self.scene_episode is not None:
            return len(self.scene_rescued_ids)
        # ROS emits the rescue event; Unity then silences the nearest physical
        # victim and acknowledges it through its remaining count. Use the
        # larger monotonic value so neither side has to wait for the other.
        if self.remaining_victims is not None:
            unity_confirmed = max(
                0, self.expected_victim_count - self.remaining_victims
            )
            return min(
                self.expected_victim_count,
                max(self.call_rescued_count, unity_confirmed),
            )
        # Before Unity's first remaining-count message, retain a bounded
        # persisted value so the HUD does not briefly jump to an invalid count.
        return min(self.expected_victim_count, self.call_rescued_count)

    def remember_rescued_position(self, x, y, z):
        if not any(
            math.hypot(x - old_x, y - old_y) <= self.rescued_suppression_distance
            for old_x, old_y, _old_z in self.rescued_positions
        ):
            self.rescued_positions.append((x, y, z))
        rospy.set_param(
            "/victim_mission_persisted_rescued_positions",
            [list(position) for position in self.rescued_positions],
        )

    def publish_state(self, status, state=None):
        if state is not None:
            self.state = state
        self.active_publisher.publish(Bool(data=self.active))
        self.status_publisher.publish(String(data=status))
        self.state_publisher.publish(String(data=self.state))
        rescued_count = self.total_rescued_count()
        rospy.set_param(
            "/victim_mission_persisted_rescued_count", rescued_count
        )
        self.rescued_count_publisher.publish(Int32(data=rescued_count))

    def start_service(self, _request):
        with self.lock:
            self.active = True
            self.mission_started_at = (
                rospy.Time.now() if self.remaining_victims is not None else None
            )
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("mission started", "LISTENING")
        return TriggerResponse(success=True, message="Victim mission started")

    def stop_service(self, _request):
        with self.lock:
            self.active = False
            self.arrival_started_at = None
            self.call_contact_started_at = None
            self.detour_goal = None
            self.detour_locked_side = None
            self.detour_side_lock_until = rospy.Time(0)
            self.stop_publisher.publish(Int8(data=2))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state("mission stopped", "IDLE")
        return TriggerResponse(success=True, message="Victim mission stopped")

    def mark_rescued_service(self, _request):
        return TriggerResponse(success=False, message="Rescue must be confirmed by Unity trigger")
        with self.lock:
            target = next(
                (track for track in self.tracks
                 if track.track_id == self.current_target_id),
                None,
            )
            if target is None:
                return TriggerResponse(success=False, message="No current victim target")
            target.rescued = True
            self.call_rescued_count = min(
                self.expected_victim_count,
                self.call_rescued_count + 1,
            )
            self.rescue_zone_armed = False
            self.contact_rescue_pending_remaining = self.remaining_victims
            self.remember_rescued_position(target.x, target.y, target.z)
            self.current_target_id = None
            self.last_goal_position = None
            self.last_goal_time = rospy.Time(0)
            self.arrival_started_at = None
            self.call_contact_started_at = None
            self.publish_state(
                "current victim marked rescued", "VICTIM_RESCUED"
            )
        return TriggerResponse(success=True, message="Current victim marked rescued")

    def reset_service(self, _request):
        with self.lock:
            # The Unity button means "reset and run the next trial", not
            # merely "erase bookkeeping". COMPLETE sets active=False, so a
            # reset that preserves that flag leaves ROS correctly cleared but
            # permanently IDLE until somebody calls the separate start
            # service. Reactivate here before the reloaded scene publishes its
            # fresh episode snapshot and goal.
            self.active = True
            self.test_handed_off_to_victim = False
            self.test_goal = None
            self.test_started = None
            self.test_arrived = False
            self.test_intermediate = None
            self.clear_committed_target()
            self.tracks = []
            self.next_track_id = 1
            self.current_target_id = None
            self.last_goal_position = None
            self.arrival_started_at = None
            self.call_contact_started_at = None
            self.call_rescued_count = 0
            self.mission_started_at = rospy.Time.now()
            rospy.set_param("/victim_mission_persisted_rescued_count", 0)
            self.rescued_positions = []
            rospy.set_param("/victim_mission_persisted_rescued_positions", [])
            self.contact_rescue_pending_remaining = None
            self.in_rescue_range = False
            self.rescue_zone_armed = True
            self.progress_anchor_position = self.robot_position
            self.progress_anchor_time = rospy.Time.now()
            self.best_call_distance = None
            self.call_progress_time = rospy.Time.now()
            self.detour_goal = None
            self.detour_started_at = rospy.Time(0)
            self.detour_last_publish = rospy.Time(0)
            self.detour_best_remaining = None
            self.detour_progress_time = rospy.Time(0)
            self.detour_side = 1.0
            self.detour_locked_side = None
            self.detour_side_lock_until = rospy.Time(0)
            self.last_call_goal_position = None
            self.last_call_goal_time = rospy.Time(0)
            self.visited_positions = (
                [(self.robot_position[0], self.robot_position[1])]
                if self.robot_position is not None
                else []
            )
            self.coverage_cells = {}
            self.coverage_obstacle_cells = set()
            self.coverage_scan_time = rospy.Time(0)
            self._exploration_progress = None
            self.last_coverage_gain_time = rospy.Time.now()
            self.loop_trajectory.clear()
            self.loop_cooldown_until = rospy.Time(0)
            self.navigation_guard_reason = "unknown"
            self.fire_boundary_since = None
            self.wall_blocked_since = None
            self.wall_replan_cooldown_until = rospy.Time(0)
            self.fire_replan_cooldown_until = rospy.Time(0)
            self.visual_bearing = None
            self.visual_world_angle = None
            self.visual_last_stamp = rospy.Time(0)
            self.clear_visual_guidance()
            self.replan_request_pending = self.replan_required
            self.recovery_exploration_until = rospy.Time(0)
            self.visual_confirmations = 0
            self.visual_received_at = rospy.Time(0)
            self.visual_rescued_seen_at = rospy.Time(0)
            self.call_precise_position = None
            self.call_precise_received_at = rospy.Time(0)
            self.failed_exploration_goals = []
            self.failed_exploration_pockets = []
            # Reset starts a fresh trial.  Do not leave the previous COMPLETE
            # stop latched, otherwise FAR can publish a valid new goal while
            # the safety arbiter continues forcing cmd_vel to zero forever.
            self.stop_publisher.publish(Int8(data=0))
            self.turn_rate_publisher.publish(Float32(data=0.0))
            self.publish_state(
                "mission reset", "LISTENING" if self.active else "IDLE"
            )
        return EmptyResponse()


def main():
    rospy.init_node("victim_mission_manager")
    VictimMissionManager()
    rospy.spin()


if __name__ == "__main__":
    main()
