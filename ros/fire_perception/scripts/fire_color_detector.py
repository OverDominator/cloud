#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from std_msgs.msg import Bool, Float32


class FireColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.stamped_roi_publisher = rospy.Publisher("/fire_detection/stamped_roi", CameraInfo, queue_size=1)

        image_topic = rospy.get_param("~image_topic", "/thermal/image_raw")
        detected_topic = rospy.get_param(
            "~detected_topic", "/fire_detection/detected"
        )
        center_topic = rospy.get_param(
            "~center_topic", "/fire_detection/center_pixel"
        )
        area_topic = rospy.get_param("~area_topic", "/fire_detection/area_pixels")
        debug_topic = rospy.get_param(
            "~debug_image_topic", "/fire_detection/debug_image"
        )
        mask_topic = rospy.get_param("~mask_topic", "/fire_detection/mask")
        roi_topic = rospy.get_param("~roi_topic", "/fire_detection/roi")

        self.lower_red_1 = self._hsv_param("~red_low_1", [0, 100, 70])
        self.upper_red_1 = self._hsv_param("~red_high_1", [12, 255, 255])
        self.lower_red_2 = self._hsv_param("~red_low_2", [168, 100, 70])
        self.upper_red_2 = self._hsv_param("~red_high_2", [179, 255, 255])
        self.min_area = float(rospy.get_param("~min_area_pixels", 120.0))
        self.kernel_size = max(1, int(rospy.get_param("~morphology_kernel", 5)))
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1
        self.morphology_iterations = max(
            0, int(rospy.get_param("~morphology_iterations", 1))
        )
        self.publish_mask = bool(rospy.get_param("~publish_mask", True))

        self.detected_publisher = rospy.Publisher(
            detected_topic, Bool, queue_size=1
        )
        self.center_publisher = rospy.Publisher(
            center_topic, PointStamped, queue_size=1
        )
        self.area_publisher = rospy.Publisher(area_topic, Float32, queue_size=1)
        self.debug_publisher = rospy.Publisher(debug_topic, Image, queue_size=1)
        self.mask_publisher = rospy.Publisher(mask_topic, Image, queue_size=1)
        self.roi_publisher = rospy.Publisher(
            roi_topic, RegionOfInterest, queue_size=1
        )
        self.image_subscriber = rospy.Subscriber(
            image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**22,
        )

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size)
        )
        rospy.loginfo(
            "Fire colour detector ready: image=%s min_area=%.1f",
            image_topic,
            self.min_area,
        )

    @staticmethod
    def _hsv_param(name, default):
        values = rospy.get_param(name, default)
        if len(values) != 3:
            raise ValueError("{} must contain exactly three HSV values".format(name))
        return np.array(values, dtype=np.uint8)

    def image_callback(self, message):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "Fire detector image conversion failed: %s", error)
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1),
            cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2),
        )

        if self.morphology_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                self.kernel,
                iterations=self.morphology_iterations,
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                self.kernel,
                iterations=self.morphology_iterations,
            )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid_contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) >= self.min_area
        ]

        detected = bool(valid_contours)
        self.detected_publisher.publish(Bool(data=detected))

        debug_image = bgr_image.copy()
        if detected:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest_contour))
            x, y, width, height = cv2.boundingRect(largest_contour)

            moments = cv2.moments(largest_contour)
            if moments["m00"] > 0.0:
                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]
            else:
                center_x = x + width / 2.0
                center_y = y + height / 2.0

            center_message = PointStamped()
            center_message.header = message.header
            center_message.header.frame_id = "thermal_image_pixels"
            center_message.point.x = center_x
            center_message.point.y = center_y
            center_message.point.z = 0.0
            self.center_publisher.publish(center_message)
            self.area_publisher.publish(Float32(data=area))
            stamped_roi = CameraInfo()
            stamped_roi.header = message.header
            stamped_roi.width = message.width
            stamped_roi.height = message.height
            stamped_roi.roi = RegionOfInterest(x_offset=x, y_offset=y, height=height, width=width)
            self.stamped_roi_publisher.publish(stamped_roi)
            self.roi_publisher.publish(
                RegionOfInterest(
                    x_offset=x,
                    y_offset=y,
                    height=height,
                    width=width,
                    do_rectify=False,
                )
            )

            cv2.rectangle(
                debug_image,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2,
            )
            cv2.drawMarker(
                debug_image,
                (int(round(center_x)), int(round(center_y))),
                (0, 255, 255),
                cv2.MARKER_CROSS,
                12,
                2,
            )
            cv2.putText(
                debug_image,
                "FIRE area={:.0f}".format(area),
                (max(0, x), max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        else:
            self.area_publisher.publish(Float32(data=0.0))
            self.roi_publisher.publish(RegionOfInterest())
            empty_roi = CameraInfo()
            empty_roi.header = message.header
            self.stamped_roi_publisher.publish(empty_roi)
            cv2.putText(
                debug_image,
                "NO FIRE",
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

        if self.publish_mask:
            mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_message.header = message.header
            self.mask_publisher.publish(mask_message)


def main():
    rospy.init_node("fire_color_detector")
    FireColorDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
