import rospy
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge

import os
import numpy as np

class FakePublisher(object):
    def __init__(self):
        self.image = cv2.imread("path/to/image")
        self.bridge = CvBridge()
        self.loop_rate = rospy.Rate(5)
        self.pub = rospy.Publisher('/image_send', Image,queue_size=10)

    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            if self.image is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = FakePublisher()
    my_node.start()