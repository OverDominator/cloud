#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


rospy.init_node("victim_hsv_probe", anonymous=True, disable_signals=True)
message = rospy.wait_for_message("/thermal/image_raw", Image, timeout=5.0)
image = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

tests = (
    ("current", (20, 100, 80), (38, 255, 255)),
    ("broad", (15, 50, 30), (45, 255, 255)),
)
print("shape={}".format(image.shape))
for name, lower, upper in tests:
    mask = cv2.inRange(
        hsv,
        np.array(lower, dtype=np.uint8),
        np.array(upper, dtype=np.uint8),
    )
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    maximum_area = max((cv2.contourArea(item) for item in contours), default=0)
    print(
        "{} pixels={} max_area={:.1f}".format(
            name, int(np.count_nonzero(mask)), maximum_area
        )
    )
