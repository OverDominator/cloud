#!/usr/bin/env python3
import math
import json
import threading

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker, MarkerArray


class FireKeepoutCloud:
    def __init__(self):
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.radius = float(rospy.get_param("~keepout_radius", 2.0))
        self.spacing = max(
            0.05, float(rospy.get_param("~keepout_point_spacing", 0.25))
        )
        self.height_levels = [
            float(value)
            for value in rospy.get_param(
                "~keepout_height_levels", [0.15, 0.55, 0.95]
            )
        ]
        self.merge_distance = float(
            rospy.get_param("~keepout_merge_distance", 1.0)
        )
        self.max_zones = max(1, int(rospy.get_param("~max_keepout_zones", 1)))
        self.require_simulated_truth = bool(rospy.get_param(
            "~require_simulated_fire_truth", False))
        self.update_alpha = float(rospy.get_param("~keepout_update_alpha", 0.2))
        self.intensity = float(rospy.get_param("~keepout_intensity", 1.0))
        self.vehicle_height = float(
            rospy.get_param("~keepout_vehicle_height", 0.75)
        )
        self.failed_pocket_radius = max(
            0.2, float(rospy.get_param("~failed_pocket_radius", 0.9))
        )
        self.failed_pocket_spacing = max(
            0.1, float(rospy.get_param("~failed_pocket_point_spacing", 0.3))
        )
        self.failed_pocket_merge_distance = max(
            0.2, float(rospy.get_param("~failed_pocket_merge_distance", 1.5))
        )
        self.failed_pocket_confirmations = max(
            1, int(rospy.get_param("~failed_pocket_confirmations", 2))
        )
        self.failed_pocket_max_zones = max(
            1, int(rospy.get_param("~failed_pocket_max_zones", 30))
        )
        self.failed_pocket_lifetime = max(
            1.0, float(rospy.get_param("~failed_pocket_lifetime", 1800.0))
        )
        self.failed_pocket_min_robot_distance = max(
            0.0,
            float(rospy.get_param("~failed_pocket_min_robot_distance", 0.45)),
        )
        self.ground_z = None
        self.robot_xy = None
        publish_rate = float(rospy.get_param("~keepout_publish_rate", 5.0))

        self.zones = []
        self.layout_fire_positions = None
        # [x, y, ground_z, confirmation_count, last_seen]
        self.failure_zones = []
        self.lock = threading.Lock()
        rospy.Subscriber('/experiment/layout', String, self.layout_callback, queue_size=1)
        self.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]

        self.keepout_publisher = rospy.Publisher(
            "/fire_detection/keepout_cloud", PointCloud2, queue_size=1
        )
        self.added_obstacles_publisher = rospy.Publisher(
            rospy.get_param("~added_obstacles_topic", "/added_obstacles"),
            PointCloud2,
            queue_size=1,
        )
        self.merged_terrain_publisher = rospy.Publisher(
            rospy.get_param(
                "~merged_terrain_topic", "/terrain_map_ext_with_fire"
            ),
            PointCloud2,
            queue_size=1,
        )
        self.marker_publisher = rospy.Publisher(
            "/fire_detection/keepout_markers", MarkerArray, queue_size=1
        )
        self.failed_pocket_cloud_publisher = rospy.Publisher(
            "/local_recovery/failed_pocket_cloud", PointCloud2, queue_size=1
        )
        self.failed_pocket_marker_publisher = rospy.Publisher(
            "/local_recovery/failed_pocket_markers", MarkerArray, queue_size=1
        )

        rospy.Subscriber(
            "/fire_detection/map_point",
            PointStamped,
            self.fire_point_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/fire_detection/simulated_truth",
            PointStamped,
            self.simulated_truth_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/state_estimation"),
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~terrain_cloud_topic", "/terrain_map_ext"),
            PointCloud2,
            self.terrain_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~failed_pocket_topic", "/local_recovery/failed_pocket"
            ),
            PointStamped,
            self.failed_pocket_callback,
            queue_size=5,
        )
        rospy.Service(
            "/fire_detection/clear_keepouts", Empty, self.clear_keepouts
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.1, publish_rate)), self.timer_callback
        )
        rospy.loginfo(
            "Fire keepout manager ready: fire radius=%.2f m, failed-pocket "
            "radius=%.2f m after %d confirmations",
            self.radius,
            self.failed_pocket_radius,
            self.failed_pocket_confirmations,
        )

    def layout_callback(self, message):
        """Replace simulated zones atomically on every authored layout snapshot.

        A randomized reset is not a noisy detection of the previous scene.
        Never apply outlier rejection/nearest-neighbor merging across runs.
        """
        if not self.require_simulated_truth:
            return
        try:
            layout = json.loads(message.data)
            positions = [(float(obj['position']['z']), -float(obj['position']['x']))
                         for obj in layout['objects']
                         if obj['name'].startswith('FireSource')]
            if not all(math.isfinite(v) for p in positions for v in p):
                raise ValueError('non-finite fire position')
        except (ValueError, TypeError, KeyError) as error:
            rospy.logwarn('Invalid fire layout snapshot: %s', error)
            return
        with self.lock:
            self.layout_fire_positions = positions
            ground = self.ground_z if self.ground_z is not None else 0.0
            self.zones = [[x, y, ground] for x, y in positions]

    def odom_callback(self, message):
        self.ground_z = message.pose.pose.position.z - self.vehicle_height
        self.robot_xy = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def fire_point_callback(self, message):
        if self.require_simulated_truth:
            # Do not allow an unvalidated image/cloud association to insert a
            # navigation obstacle into a scene with known simulated fire pose.
            return
        self.store_fire_point(message, "perception")

    def simulated_truth_callback(self, message):
        if self.layout_fire_positions is not None:
            return  # Complete layout snapshot owns all simulated fire identities.
        self.store_fire_point(message, "simulation truth")

    def store_fire_point(self, message, source):
        if message.header.frame_id != self.output_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring fire point in frame %s; expected %s",
                message.header.frame_id,
                self.output_frame,
            )
            return

        if self.ground_z is None:
            rospy.logwarn_throttle(2.0, "Waiting for odometry before storing fire zone")
            return

        # Camera/LiDAR fusion may hit any height on the fire object. A keepout
        # zone must start on the traversable floor to survive planner Z filters.
        point = [message.point.x, message.point.y, self.ground_z]
        with self.lock:
            nearest_index = None
            nearest_distance = float("inf")
            for index, zone in enumerate(self.zones):
                distance = math.hypot(point[0] - zone[0], point[1] - zone[1])
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_index = index

            if nearest_index is not None and nearest_distance <= self.merge_distance:
                zone = self.zones[nearest_index]
                alpha = self.update_alpha
                self.zones[nearest_index] = [
                    (1.0 - alpha) * zone[0] + alpha * point[0],
                    (1.0 - alpha) * zone[1] + alpha * point[1],
                    (1.0 - alpha) * zone[2] + alpha * point[2],
                ]
            elif len(self.zones) >= self.max_zones:
                rospy.logwarn_throttle(
                    2.0,
                    "Rejected fire localization outlier at (%.2f, %.2f), "
                    "distance from confirmed zone=%.2f m",
                    point[0],
                    point[1],
                    nearest_distance,
                )
            else:
                self.zones.append(point)
                rospy.logwarn(
                    "New %s fire keepout zone %d at (%.2f, %.2f), radius=%.2f m",
                    source, len(self.zones),
                    point[0],
                    point[1],
                    self.radius,
                )

    def failed_pocket_callback(self, message):
        if message.header.frame_id not in (self.output_frame, "/" + self.output_frame):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring failed pocket in frame %s; expected %s",
                message.header.frame_id,
                self.output_frame,
            )
            return
        if self.ground_z is None:
            rospy.logwarn_throttle(
                2.0, "Waiting for odometry before storing failed pocket"
            )
            return

        now = rospy.Time.now()
        point = [message.point.x, message.point.y, self.ground_z]
        if (
            self.robot_xy is not None
            and math.hypot(
                point[0] - self.robot_xy[0], point[1] - self.robot_xy[1]
            ) < self.failed_pocket_min_robot_distance
        ):
            rospy.logerr_throttle(
                2.0,
                "Rejected failed-pocket keepout at robot centre (%.2f, %.2f)",
                point[0], point[1],
            )
            return
        with self.lock:
            self._prune_failure_zones_locked(now)
            nearest_index = None
            nearest_distance = float("inf")
            for index, zone in enumerate(self.failure_zones):
                distance = math.hypot(point[0] - zone[0], point[1] - zone[1])
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_index = index

            if (
                nearest_index is not None
                and nearest_distance <= self.failed_pocket_merge_distance
            ):
                zone = self.failure_zones[nearest_index]
                count = int(zone[3]) + 1
                alpha = min(0.35, 1.0 / float(count))
                zone[0] = (1.0 - alpha) * zone[0] + alpha * point[0]
                zone[1] = (1.0 - alpha) * zone[1] + alpha * point[1]
                zone[2] = point[2]
                zone[3] = count
                zone[4] = now
                if count == self.failed_pocket_confirmations:
                    rospy.logwarn(
                        "Repeated navigation failure at (%.2f, %.2f); "
                        "injecting temporary planner keepout radius %.2f m",
                        zone[0], zone[1], self.failed_pocket_radius,
                    )
            elif len(self.failure_zones) < self.failed_pocket_max_zones:
                self.failure_zones.append(
                    [point[0], point[1], point[2], 1, now]
                )
                rospy.logwarn(
                    "Remembered first navigation failure at (%.2f, %.2f); "
                    "waiting for confirmation before hard keepout",
                    point[0], point[1],
                )

    def _prune_failure_zones_locked(self, now):
        self.failure_zones = [
            zone for zone in self.failure_zones
            if (now - zone[4]).to_sec() < self.failed_pocket_lifetime
        ]

    @staticmethod
    def _disc_points(zones, radius, spacing, height_levels, intensity):
        points = []
        radial_steps = max(1, int(math.ceil(radius / spacing)))
        for center_x, center_y, center_z in zones:
            for radial_index in range(radial_steps + 1):
                radial = min(radius, radial_index * spacing)
                circumference = 2.0 * math.pi * max(radial, spacing)
                angle_steps = max(8, int(math.ceil(circumference / spacing)))
                for angle_index in range(angle_steps):
                    angle = 2.0 * math.pi * angle_index / angle_steps
                    x = center_x + radial * math.cos(angle)
                    y = center_y + radial * math.sin(angle)
                    for height in height_levels:
                        points.append((x, y, center_z + height, intensity))
        return points

    def zone_points(self):
        with self.lock:
            zones = [zone[:] for zone in self.zones]

        points = self._disc_points(
            zones, self.radius, self.spacing, self.height_levels, self.intensity
        )
        return points, zones

    def failure_zone_points(self):
        now = rospy.Time.now()
        with self.lock:
            self._prune_failure_zones_locked(now)
            zones = [
                zone[:3] for zone in self.failure_zones
                if zone[3] >= self.failed_pocket_confirmations
            ]
        points = self._disc_points(
            zones,
            self.failed_pocket_radius,
            self.failed_pocket_spacing,
            self.height_levels,
            self.intensity,
        )
        return points, zones

    def make_cloud(self, points, stamp=None):
        header = Header()
        header.stamp = stamp if stamp is not None else rospy.Time.now()
        header.frame_id = self.output_frame
        return point_cloud2.create_cloud(header, self.fields, points)

    def timer_callback(self, _event):
        fire_points, fire_zones = self.zone_points()
        failure_points, failure_zones = self.failure_zone_points()
        self.keepout_publisher.publish(self.make_cloud(fire_points))
        self.failed_pocket_cloud_publisher.publish(
            self.make_cloud(failure_points)
        )
        self.added_obstacles_publisher.publish(
            # Fire is an immediate local hazard. Failed corridors are a
            # high-level route memory only; feeding them to localPlanner can
            # immobilise a robot that has not yet finished retreating.
            self.make_cloud(fire_points)
        )
        self.publish_markers(fire_zones, failure_zones)

    def terrain_callback(self, message):
        keepout_points, _ = self.zone_points()
        failure_points, _ = self.failure_zone_points()
        try:
            terrain_points = list(
                point_cloud2.read_points(
                    message,
                    field_names=("x", "y", "z", "intensity"),
                    skip_nans=True,
                )
            )
        except ValueError as error:
            rospy.logerr_throttle(2.0, "Cannot merge terrain cloud: %s", error)
            return

        merged = self.make_cloud(
            terrain_points + keepout_points + failure_points,
            stamp=message.header.stamp,
        )
        self.merged_terrain_publisher.publish(merged)

    def publish_markers(self, zones, failure_zones):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        for index, zone in enumerate(zones):
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "fire_keepout"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = zone[0]
            marker.pose.position.y = zone[1]
            marker.pose.position.z = zone[2] + 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self.radius
            marker.scale.y = 2.0 * self.radius
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.0
            marker.color.a = 0.3
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

        failed_markers = MarkerArray()
        clear_failed = Marker()
        clear_failed.action = Marker.DELETEALL
        failed_markers.markers.append(clear_failed)
        for index, zone in enumerate(failure_zones):
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "failed_navigation_pocket"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = zone[0]
            marker.pose.position.y = zone[1]
            marker.pose.position.z = zone[2] + 0.04
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self.failed_pocket_radius
            marker.scale.y = 2.0 * self.failed_pocket_radius
            marker.scale.z = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.0
            marker.color.a = 0.45
            failed_markers.markers.append(marker)
        self.failed_pocket_marker_publisher.publish(failed_markers)

    def clear_keepouts(self, _request):
        with self.lock:
            self.zones = []
            self.failure_zones = []
        rospy.logwarn("All stored fire and failed-pocket keepout zones cleared")
        return EmptyResponse()


def main():
    rospy.init_node("fire_keepout_cloud")
    FireKeepoutCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
