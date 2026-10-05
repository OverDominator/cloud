#!/usr/bin/env python3
import math

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, RegionOfInterest, CameraInfo
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class FirePointCloudLocalizer:
    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", "thermal_camera_link")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.image_width = float(rospy.get_param("~image_width", 320))
        self.image_height = float(rospy.get_param("~image_height", 180))
        vertical_fov = math.radians(
            float(rospy.get_param("~vertical_fov_degrees", 60.0))
        )
        self.fy = self.image_height / (2.0 * math.tan(vertical_fov / 2.0))
        self.fx = self.fy  # Unity perspective camera: square pixels.
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        self.roi_margin = int(rospy.get_param("~roi_margin_pixels", 5))
        self.roi_timeout = float(rospy.get_param("~roi_timeout", 0.15))
        self.min_range = float(rospy.get_param("~min_range", 0.4))
        self.max_range = float(rospy.get_param("~max_range", 30.0))
        self.depth_band = float(
            rospy.get_param("~front_surface_depth_band", 0.8)
        )
        # A two-point projection from the sparse 4-degree LiDAR can easily
        # be a wall edge behind the coloured ROI.  Do not create a navigation
        # obstacle unless several independent returns support it.
        self.min_lidar_points = max(
            3, int(rospy.get_param("~min_fire_lidar_points", 4))
        )
        self.vehicle_height = float(rospy.get_param("~keepout_vehicle_height", 0.75))
        self.min_height_above_ground = float(
            rospy.get_param("~min_fire_height_above_ground", 0.35)
        )
        self.ground_z = None
        self.marker_scale = float(rospy.get_param("~marker_scale", 0.6))

        self.roi = None
        self.roi_stamp = rospy.Time(0)
        self.camera_info_ready = False
        self.roi_received_at = rospy.Time(0)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/thermal/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        self.point_publisher = rospy.Publisher(
            "/fire_detection/map_point", PointStamped, queue_size=1
        )
        self.valid_publisher = rospy.Publisher(
            "/fire_detection/localization_valid", Bool, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/fire_detection/map_marker", Marker, queue_size=1
        )

        rospy.Subscriber(
            "/fire_detection/stamped_roi",
            CameraInfo,
            self.roi_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~pointcloud_topic", "/registered_scan"),
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/state_estimation"),
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "Fire point-cloud localizer ready: fx=%.2f fy=%.2f camera=%s",
            self.fx,
            self.fy,
            self.camera_frame,
        )

    def odom_callback(self, message):
        self.ground_z = message.pose.pose.position.z - self.vehicle_height

    def camera_info_callback(self, message):
        if message.width <= 0 or message.height <= 0 or message.K[0] <= 0 or message.K[4] <= 0:
            return
        self.image_width, self.image_height = float(message.width), float(message.height)
        self.fx, self.fy = message.K[0], message.K[4]
        self.cx, self.cy = message.K[2], message.K[5]
        self.camera_info_ready = True

    def roi_callback(self, message):
        self.roi_stamp = message.header.stamp
        self.roi = message.roi if message.roi.width > 0 and message.roi.height > 0 else None
        self.roi_received_at = rospy.Time.now()
        if self.roi is None:
            self.valid_publisher.publish(Bool(data=False))

    @staticmethod
    def transform_matrix(translation, quaternion):
        matrix = tf.transformations.quaternion_matrix(quaternion)
        matrix[0:3, 3] = np.asarray(translation)
        return matrix

    def cloud_callback(self, message):
        if self.roi is None or not self.camera_info_ready:
            return
        if self.roi_stamp.to_sec() <= 0 or abs((message.header.stamp - self.roi_stamp).to_sec()) > 0.15:
            self.valid_publisher.publish(Bool(data=False))
            return
        if (rospy.Time.now() - self.roi_received_at).to_sec() > self.roi_timeout:
            self.valid_publisher.publish(Bool(data=False))
            return

        try:
            # TF is published at 10 Hz and may arrive just after the image.
            # Wait briefly for the capture pose; never substitute latest pose.
            roi = self.roi
            roi_stamp = self.roi_stamp
            self.listener.waitForTransform(self.camera_frame, message.header.frame_id,
                                           roi_stamp, rospy.Duration(0.25))
            translation, quaternion = self.listener.lookupTransform(
                self.camera_frame,
                message.header.frame_id,
                roi_stamp,
            )
            camera_from_cloud = self.transform_matrix(translation, quaternion)
            output_translation, output_quaternion = self.listener.lookupTransform(
                self.output_frame,
                message.header.frame_id,
                message.header.stamp,
            )
            output_from_cloud = self.transform_matrix(
                output_translation, output_quaternion
            )
        except tf.Exception as error:
            rospy.logwarn_throttle(2.0, "Fire localization TF unavailable: %s", error)
            self.valid_publisher.publish(Bool(data=False))
            return

        points = np.asarray(
            list(
                point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
            ),
            dtype=np.float64,
        )
        if points.size == 0:
            self.valid_publisher.publish(Bool(data=False))
            return

        homogeneous = np.column_stack((points, np.ones(points.shape[0])))
        camera_points = (camera_from_cloud @ homogeneous.T).T[:, :3]

        # thermal_camera_link uses ROS FLU: X forward, Y left, Z up.
        forward = camera_points[:, 0]
        valid_depth = (forward >= self.min_range) & (forward <= self.max_range)
        u = self.cx - self.fx * camera_points[:, 1] / np.maximum(forward, 1e-6)
        v = self.cy - self.fy * camera_points[:, 2] / np.maximum(forward, 1e-6)

        x_min = max(0, int(roi.x_offset) - self.roi_margin)
        y_min = max(0, int(roi.y_offset) - self.roi_margin)
        x_max = min(
            int(self.image_width), int(roi.x_offset + roi.width) + self.roi_margin
        )
        y_max = min(
            int(self.image_height), int(roi.y_offset + roi.height) + self.roi_margin
        )
        inside = (
            valid_depth
            & (u >= x_min)
            & (u <= x_max)
            & (v >= y_min)
            & (v <= y_max)
        )
        candidate_indices = np.flatnonzero(inside)
        if candidate_indices.size < self.min_lidar_points:
            rospy.logwarn_throttle(
                2.0,
                "Fire ROI has only %d LiDAR points; need %d before adding a keepout",
                candidate_indices.size,
                self.min_lidar_points,
            )
            self.valid_publisher.publish(Bool(data=False))
            return

        candidate_depths = forward[candidate_indices]
        front_depth = float(np.percentile(candidate_depths, 10.0))
        surface_indices = candidate_indices[
            candidate_depths <= front_depth + self.depth_band
        ]
        output_points = (output_from_cloud @ homogeneous[surface_indices].T).T[:, :3]
        if self.ground_z is not None:
            output_points = output_points[
                output_points[:, 2] >= self.ground_z + self.min_height_above_ground
            ]
        if output_points.shape[0] < self.min_lidar_points:
            rospy.logwarn_throttle(
                2.0,
                "Fire ROI lacks elevated LiDAR support; rejecting ground/background points",
            )
            self.valid_publisher.publish(Bool(data=False))
            return
        fire_point = np.median(output_points, axis=0)

        point_message = PointStamped()
        point_message.header.stamp = rospy.Time.now()
        point_message.header.frame_id = self.output_frame
        point_message.point.x = float(fire_point[0])
        point_message.point.y = float(fire_point[1])
        point_message.point.z = float(fire_point[2])
        self.point_publisher.publish(point_message)
        self.valid_publisher.publish(Bool(data=True))
        self.publish_marker(point_message)

        rospy.loginfo_throttle(
            2.0,
            "Fire localized at (%.2f, %.2f, %.2f), ROI points=%d",
            fire_point[0],
            fire_point[1],
            fire_point[2],
            surface_indices.size,
        )

    def publish_marker(self, point_message):
        marker = Marker()
        marker.header = point_message.header
        marker.ns = "fire_detection"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_message.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime = rospy.Duration(0.5)
        self.marker_publisher.publish(marker)


def main():
    rospy.init_node("fire_pointcloud_localizer")
    FirePointCloudLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
