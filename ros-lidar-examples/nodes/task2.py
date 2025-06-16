#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import math
import statistics
from typing import List, Optional, Tuple
from scipy.signal import medfilt

import rospy
from sensor_msgs.msg import LaserScan
from typing import Final


ROS_SCAN_TOPIC: Final[str] = "/scan2"


class ObstacleWarn:
  def __init__(self):
    rospy.init_node('obstacle_warn')
    self.sub = rospy.Subscriber(ROS_SCAN_TOPIC, LaserScan, self.scan_callback,queue_size=None)


  def scan_callback(self, msg: LaserScan) -> None:
    print("Hello")
    self.last_msg = msg
    ranges_filtered = []
    spread = math.radians(2.5) / msg.angle_increment
    center = int((0 - msg.angle_min) / msg.angle_increment)

    minIndex = int(max(0, center - spread))
    maxIndex = int(min(len(msg.ranges) - 1, center + spread))

    
    for laser in msg.ranges[minIndex:maxIndex]:
        if  not laser == math.isnan and not laser == math.isinf:
          ranges_filtered.append(laser)
        elif laser == math.isinf:
          ranges_filtered.append(msg.range_max)
    avg = sum(ranges_filtered) / len(ranges_filtered)
    if avg < 0.5:
      rospy.logwarn("ОПАСНОЕ ПРИБЛИЖЕНИЕ!")



def main() -> None:
  detector = ObstacleWarn()
  rospy.spin()

if __name__ == '__main__':
  main()
