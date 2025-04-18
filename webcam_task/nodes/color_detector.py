#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg
import numpy as np
import sys

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "color_detector_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC: Final[str] = "/usb_cam/image_raw"
ROS_MASK_TOPIC: Final[str] = "/color_diff"



class ColorDetector:
    def __init__(self, color):
        rospy.init_node(ROS_NODE_NAME)
        rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
        rospy.loginfo(f"OpenCV version: {cv2.__version__}")

        # wait for the images to arrive or throw an exception
        self.sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=3.0)
        self.cv_bridge: CvBridge = CvBridge()
        self.pub = rospy.Publisher(ROS_MASK_TOPIC, Image, queue_size=10)
        self.sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self.image_callback, queue_size=None)
        self.color = color
        if self.sample is not None:
            rospy.loginfo(f"Encoding: {self.sample.encoding}, Resolution: {self.sample.width, self.sample.height}")
        
    def image_callback(self, msg):
        
        image = self.cv_bridge.imgmsg_to_cv2(msg)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(image_hsv, self.color)        
        image_masked = cv2.bitwise_and(image, image, mask=mask)
        image_rgb = cv2.cvtColor(image_masked, cv2.COLOR_BGR2RGB)
        final_img = self.cv_bridge.cv2_to_imgmsg(image_rgb, "mono8")
        self.pub.publish(final_img)

    def get_mask(image_hsv, color):
        lower_red_lower = np.array([0, 70, 50])
        lower_red_upper = np.array([10, 255, 255])
        upper_red_lower = np.array([170, 70, 50])
        upper_red_upper = np.array([180, 255, 255])

        upper_blue = np.array([128, 255, 255])
        lower_blue = np.array([90, 50, 70])

        lower_green = np.array([20, 100, 100])
        upper_green = np.array([70, 255, 255])
        
        if color == "красный":
            mask_lower_red = cv2.inRange(image_hsv, lower_red_lower, lower_red_upper)
            mask_upper_red = cv2.inRange(image_hsv, upper_red_lower, upper_red_upper)
            return cv2.bitwise_or(mask_lower_red, mask_upper_red)
        elif color == "синий":
            return cv2.inRange(image_hsv, lower_blue, upper_blue)
        elif color == "зеленый":
            return cv2.inRange(image_hsv, lower_green, upper_green)


if __name__ == '__main__':
    try:
        if len(sys.argv) < 1:
            print("Not enough arguments!")
            sys.exit(1)
        value = str(sys.argv[1])
        ColorDetector(color=value)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass