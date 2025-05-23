#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import math

class GarbageRobot:
    def __init__(self):
        rospy.init_node('garbage_robot')

        # Parameters
        self.target_distance = rospy.get_param('~target_distance', 1.0)  # meters
        self.forward_speed  = rospy.get_param('~forward_speed', 0.25)     # m/s
        self.turn_gain      = rospy.get_param('~turn_gain', 0.003)       # rad/pixel
        self.search_turn_speed = rospy.get_param('~search_turn_speed', 0.1)

        # State
        self.bridge = CvBridge()
        self.image_width = None
        self.object_center_x = None
        self.green_detected = False
        self.distance_ahead = float('inf')

        self.scan_sub  = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=10)
        self.lidar_pub  = rospy.Publisher('/test_lidar', LaserScan, queue_size=1)

        self.rate = rospy.Rate(10)


    def scan_cb(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

    # Define the frontal 30 degree window (±15 degrees)
        front_angle_min = -math.radians(15)
        front_angle_max = math.radians(15)

    # Compute index range
        index_min = int((front_angle_min - angle_min) / angle_increment)
        index_max = int((front_angle_max - angle_min) / angle_increment)
        index_min = 360 - abs(index_min)
        # print(ranges)
        # print(index_max)

        front_left_ranges = ranges[index_min:360]
        front_right_ranges = ranges[0:index_max+1]
        front_ranges = front_left_ranges + front_right_ranges

    # You can now use these values, e.g., find minimum distance in front
        front_distances = [r for r in front_ranges if not math.isnan(r) and r > 0]
        if front_distances:
            min_front = min(front_distances)
            print(f"Min distance in front 30°: {min_front:.2f} m")
        else:
            print("No valid measurements in front sector.")

        
        send_msg = msg
        send_msg.ranges = front_ranges
        self.lidar_pub.publish(send_msg)
        print(self.distance_ahead)


    def run(self):
        rospy.loginfo("TurtleBot3 Green Can Follower started.")
        while not rospy.is_shutdown():
            twist = Twist()
            self.rate.sleep()
            rospy.spin()


if __name__ == '__main__':
    try:
        robot = GarbageRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass