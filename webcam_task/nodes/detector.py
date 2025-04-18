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

ROS_IMAGE_TOPIC: Final[str] = "/usb_cam/image_raw"
ROS_MOTION_TOPIC: Final[str] = "/usb_cam/motion_detection"


class MotionDetector:
    def __init__(self):
        rospy.init_node(ROS_NODE_NAME)
        rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
        rospy.loginfo(f"OpenCV version: {cv2.__version__}")

        self.sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=3.0)
        self.cv_bridge: CvBridge = CvBridge()
        self.sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self.image_callback, queue_size=None)
        self.pub = rospy.Publisher(ROS_MOTION_TOPIC, Motion, queue_size=10)        
        self.prev_frame = None  # Храним предыдущий кадр
        if self.sample is not None:
            rospy.loginfo(f"Encoding: {self.sample.encoding}, Resolution: {self.sample.width, self.sample.height}")
        
    def image_callback(self, msg):
    # конвертация ROS-сообщения в OpenCV изображение
    # Q: Каков формат у изображения?
    # A: Формат изображения после конвертации - cv::Mat
        image = self.cv_bridge.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.prev_frame is None:
            self.prev_frame = frame
            return
        diff = cv2.absdiff(self.prev_frame, frame)     
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)          
        motion_pixels = np.sum(thresh > 0)
        motion_detected = motion_pixels > 2500
        msg = Motion()
        msg.detected = motion_detected
        msg.diff = np.sum(diff)
        print(msg.detected)
        self.pub.publish(msg)
        self.prev_frame = frame


if __name__ == '__main__':
    try:
        MotionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass