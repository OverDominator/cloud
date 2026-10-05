#!/usr/bin/env python3
import math
from copy import deepcopy

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseArray, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, UInt32


class VictimColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lower_yellow = self._hsv_param("~yellow_low", [20, 100, 80])
        self.upper_yellow = self._hsv_param("~yellow_high", [38, 255, 255])
        self.min_area = float(rospy.get_param("~min_area_pixels", 80.0))
        self.min_component_area = float(rospy.get_param("~min_component_area_pixels", 3.0))
        self.min_target_height = int(rospy.get_param("~min_target_height_pixels", 5))
        self.min_target_aspect = float(rospy.get_param("~min_target_aspect_ratio", 1.05))
        self.min_target_fill = float(rospy.get_param("~min_target_fill_ratio", 0.2))
        self.horizontal_fov = math.radians(
            float(rospy.get_param("~horizontal_fov_degrees", 91.5))
        )
        self.camera_frame = rospy.get_param("~camera_frame", "thermal_camera_link")
        self.bearing_frame = rospy.get_param("~visual_bearing_frame", "vehicle")
        self.camera_info = None
        self.listener = tf.TransformListener()
        self.kernel_size = max(1, int(rospy.get_param("~morphology_kernel", 5)))
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1
        self.iterations = max(
            0, int(rospy.get_param("~morphology_iterations", 1))
        )
        self.merge_gap = max(0, int(rospy.get_param("~merge_gap_pixels", 18)))
        self.publish_mask = bool(rospy.get_param("~publish_mask", True))
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size)
        )

        self.detected_publisher = rospy.Publisher(
            rospy.get_param("~detected_topic", "/victim_detection/detected"),
            Bool,
            queue_size=1,
        )
        self.count_publisher = rospy.Publisher(
            rospy.get_param("~count_topic", "/victim_detection/count"),
            UInt32,
            queue_size=1,
        )
        self.centers_publisher = rospy.Publisher(
            rospy.get_param("~centers_topic", "/victim_detection/centers"),
            PoseArray,
            queue_size=1,
        )
        self.boxes_publisher = rospy.Publisher(
            rospy.get_param("~boxes_topic", "/victim_detection/boxes"),
            Float32MultiArray,
            queue_size=1,
        )
        self.boxes_stamped_publisher = rospy.Publisher(
            rospy.get_param(
                "~boxes_stamped_topic", "/victim_detection/boxes_stamped"
            ),
            PolygonStamped,
            queue_size=1,
        )
        self.visual_bearing_publisher = rospy.Publisher(
            rospy.get_param(
                "~visual_bearing_topic", "/victim_detection/visual_bearing"
            ),
            Vector3Stamped,
            queue_size=1,
        )
        self.debug_publisher = rospy.Publisher(
            rospy.get_param(
                "~debug_image_topic", "/victim_detection/debug_image"
            ),
            Image,
            queue_size=1,
        )
        self.mask_publisher = rospy.Publisher(
            rospy.get_param("~mask_topic", "/victim_detection/mask"),
            Image,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/thermal/camera_info"),
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/thermal/image_raw"),
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**22,
        )
        rospy.loginfo(
            "Victim colour detector ready: yellow HSV %s..%s min_area=%.1f",
            self.lower_yellow.tolist(),
            self.upper_yellow.tolist(),
            self.min_area,
        )

    @staticmethod
    def _hsv_param(name, default):
        values = rospy.get_param(name, default)
        if len(values) != 3:
            raise ValueError("{} must contain three HSV values".format(name))
        return np.array(values, dtype=np.uint8)

    def image_callback(self, message):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(
                2.0, "Victim detector image conversion failed: %s", error
            )
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        if self.iterations > 0:
            # Opening erodes a distant person's 2-4 pixel wide body completely.
            # Small components are rejected below; close only reconnects parts.
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.iterations
            )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid_contours = sorted(
            (
                contour
                for contour in contours
                if cv2.contourArea(contour) >= self.min_component_area
            ),
            key=cv2.contourArea,
            reverse=True,
        )
        contour_groups = [
            group for group in self._group_nearby_contours(valid_contours)
            if self._is_target_group(group)
        ]

        count = len(contour_groups)
        self.detected_publisher.publish(Bool(data=count > 0))
        self.count_publisher.publish(UInt32(data=count))

        centers = PoseArray()
        centers.header = deepcopy(message.header)
        centers.header.frame_id = "thermal_image_pixels"
        box_values = []
        stamped_boxes = PolygonStamped()
        stamped_boxes.header = deepcopy(message.header)
        stamped_boxes.header.frame_id = "thermal_image_pixels"
        debug_image = bgr_image.copy()

        for index, group in enumerate(contour_groups):
            boxes = [cv2.boundingRect(contour) for contour in group]
            x = min(box[0] for box in boxes)
            y = min(box[1] for box in boxes)
            x_max = max(box[0] + box[2] for box in boxes)
            y_max = max(box[1] + box[3] for box in boxes)
            width = x_max - x
            height = y_max - y

            areas = [float(cv2.contourArea(contour)) for contour in group]
            area = sum(areas)
            weighted_x = 0.0
            weighted_y = 0.0
            total_weight = 0.0
            for contour, contour_area in zip(group, areas):
                moments = cv2.moments(contour)
                if moments["m00"] > 0.0:
                    contour_x = moments["m10"] / moments["m00"]
                    contour_y = moments["m01"] / moments["m00"]
                else:
                    box = cv2.boundingRect(contour)
                    contour_x = box[0] + box[2] / 2.0
                    contour_y = box[1] + box[3] / 2.0
                weight = max(1.0, contour_area)
                weighted_x += contour_x * weight
                weighted_y += contour_y * weight
                total_weight += weight
            center_x = weighted_x / total_weight
            center_y = weighted_y / total_weight

            pose = Pose()
            pose.position.x = center_x
            pose.position.y = center_y
            pose.orientation.w = 1.0
            centers.poses.append(pose)
            box_values.extend([float(x), float(y), float(width), float(height), area])
            stamped_boxes.polygon.points.append(
                Point32(x=float(x), y=float(y), z=area)
            )
            stamped_boxes.polygon.points.append(
                Point32(x=float(x + width), y=float(y + height), z=0.0)
            )

            cv2.rectangle(
                debug_image, (x, y), (x + width, y + height), (255, 255, 0), 2
            )
            cv2.drawMarker(
                debug_image,
                (int(round(center_x)), int(round(center_y))),
                (0, 255, 255),
                cv2.MARKER_CROSS,
                10,
                2,
            )
            cv2.putText(
                debug_image,
                "VICTIM {}".format(index + 1),
                (max(0, x), max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )

            # The first group is the largest yellow target.  Publish a coarse
            # camera-relative bearing even when LiDAR cannot yet provide a
            # precise map position.  Vehicle convention is X forward, Y left.
            if index == 0:
                self.publish_bearing(message.header, center_x, center_y, bgr_image.shape)

        if count == 0:
            cv2.putText(
                debug_image,
                "NO VICTIM",
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        self.centers_publisher.publish(centers)
        self.boxes_publisher.publish(Float32MultiArray(data=box_values))
        self.boxes_stamped_publisher.publish(stamped_boxes)

        debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

        if self.publish_mask:
            mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_message.header = message.header
            self.mask_publisher.publish(mask_message)

    def camera_info_callback(self, message):
        if message.width > 0 and message.height > 0 and message.K[0] > 0 and message.K[4] > 0:
            self.camera_info = message

    def publish_bearing(self, header, center_x, center_y, shape):
        """Publish a direction immediately; metric depth is not a prerequisite."""
        if header.stamp.to_sec() <= 0.0:
            rospy.logwarn_throttle(2.0, "Victim image has a zero timestamp; bearing skipped")
            return
        height, width = shape[:2]
        info = self.camera_info
        if info is not None:
            scale_x, scale_y = width / float(info.width), height / float(info.height)
            fx, fy = info.K[0] * scale_x, info.K[4] * scale_y
            cx, cy = info.K[2] * scale_x, info.K[5] * scale_y
        else:
            fx = width / (2.0 * math.tan(self.horizontal_fov / 2.0))
            fy, cx, cy = fx, width / 2.0, height / 2.0
        bearing = Vector3Stamped()
        bearing.header = deepcopy(header)
        bearing.header.frame_id = self.camera_frame
        bearing.vector.x = 1.0
        bearing.vector.y = (cx - center_x) / fx
        bearing.vector.z = (cy - center_y) / fy
        try:
            if self.camera_frame != self.bearing_frame:
                # Image transport may beat TF by a few milliseconds. Wait
                # briefly for the exact capture timestamp, never substitute
                # Time(0), which would rotate old pixels using the new pose.
                self.listener.waitForTransform(
                    self.bearing_frame, self.camera_frame, header.stamp,
                    rospy.Duration(0.12))
                bearing = self.listener.transformVector3(self.bearing_frame, bearing)
        except (tf.Exception, rospy.ROSException) as error:
            rospy.logwarn_throttle(2.0, "Victim bearing camera TF unavailable: %s", error)
            return
        norm = math.hypot(bearing.vector.x, bearing.vector.y)
        if norm <= 1e-6:
            return
        bearing.vector.x /= norm
        bearing.vector.y /= norm
        bearing.vector.z = 0.0
        self.visual_bearing_publisher.publish(bearing)

    def _is_target_group(self, group):
        boxes = [cv2.boundingRect(contour) for contour in group]
        width = max(x + w for x, y, w, h in boxes) - min(x for x, y, w, h in boxes)
        height = max(y + h for x, y, w, h in boxes) - min(y for x, y, w, h in boxes)
        area = sum(cv2.contourArea(contour) for contour in group)
        return (
            area >= self.min_area
            and height >= self.min_target_height
            and height >= width * self.min_target_aspect
            and area / max(1.0, float(width * height)) >= self.min_target_fill
        )

    def _group_nearby_contours(self, contours):
        """Join vertically aligned head/body parts, not side-by-side people."""
        if not contours:
            return []

        boxes = [cv2.boundingRect(contour) for contour in contours]
        parents = list(range(len(contours)))

        def find(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        def union(first, second):
            first_root = find(first)
            second_root = find(second)
            if first_root != second_root:
                parents[second_root] = first_root

        gap = self.merge_gap
        for first in range(len(boxes)):
            x1, y1, width1, height1 = boxes[first]
            for second in range(first + 1, len(boxes)):
                x2, y2, width2, height2 = boxes[second]
                horizontal_overlap = min(x1 + width1, x2 + width2) - max(x1, x2)
                horizontally_close = horizontal_overlap >= 0.35 * min(width1, width2)
                vertical_gap = max(y1, y2) - min(y1 + height1, y2 + height2)
                vertically_close = 0 <= vertical_gap <= min(gap, max(width1, width2))
                if horizontally_close and vertically_close:
                    union(first, second)

        groups = {}
        for index, contour in enumerate(contours):
            groups.setdefault(find(index), []).append(contour)
        return sorted(
            groups.values(),
            key=lambda group: sum(cv2.contourArea(item) for item in group),
            reverse=True,
        )


def main():
    rospy.init_node("victim_color_detector")
    VictimColorDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
