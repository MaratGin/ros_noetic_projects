#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg
import numpy as np

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from std_msgs.msg import Bool

from typing import Final
from webcam_task.msg import Motion

# constants
ROS_NODE_NAME: Final[str] = "detector_subsriber"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_MOTION_TOPIC: Final[str] = "/usb_cam/motion_detection"


def image_callback(msg: Motion) -> None:
    print(msg.detected)
  

def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
  rospy.loginfo(f"OpenCV version: {cv2.__version__}")
  

  rospy.Subscriber(ROS_MOTION_TOPIC, Motion, lambda msg: image_callback(msg), queue_size=None)

  # Q: Что происходит в данной строчке кода?
  # A: rospy.spin() помогает не завершать выполнение ноды, пока она не будет отключена
  rospy.spin()


if __name__ == '__main__':
  main()