#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg
import numpy as np

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge
from webcam_task.msg import Motion

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "detector_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC: Final[str] = "/opcv"
ROS_MOTION_TOPIC: Final[str] = "/usb_cam/motion_detection"


class MotionDetector:
    def __init__(self):
        rospy.init_node(ROS_NODE_NAME)
        rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
        rospy.loginfo(f"OpenCV version: {cv2.__version__}")

        self.sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=15.0)
        self.cv_bridge: CvBridge = CvBridge()
        self.sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self.image_callback, queue_size=None)
        self.prev_frame = None  # Храним предыдущий кадр
        if self.sample is not None:
            rospy.loginfo(f"Encoding: {self.sample.encoding}, Resolution: {self.sample.width, self.sample.height}")
                   
    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([38, 18, 18])
        upper = np.array([86, 255, 255])
        mask  = cv2.inRange(hsv, lower, upper)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.green_detected = False
            return

        max_cont = max(contours, key=cv2.contourArea)
        # for contour in contours:
        (x,y,w,h) = cv2.boundingRect(max_cont)
        cv2.rectangle(mask,(x,y), (x+w,y+h), (255,255,0), 4)
        
        window = "Image"  
        final = cv2.drawContours(image, contours, -1, (0,255,0), 3)
        window = cv2.resizeWindow("Resized_Window", 720, 360) 
        res = cv2.resize(mask, dsize=(720, 360), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("window", res)
        cv2.waitKey(1)
        # self.pub.publish(msg)


if __name__ == '__main__':
    try:
        MotionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass