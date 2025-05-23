#!/usr/bin/env python3
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class GarbageRobot:
    def __init__(self):
        rospy.init_node('tb3_green_can_follower_adapted')

        # Parameters
        self.target_distance = rospy.get_param('~target_distance', 1.0)  # meters
        self.forward_speed  = rospy.get_param('~forward_speed', 0.2)     # m/s
        self.turn_gain      = rospy.get_param('~turn_gain', 0.003)       # rad/pixel
        self.search_turn_speed = rospy.get_param('~search_turn_speed', 0.2)

        # State
        self.bridge = CvBridge()
        self.image_width = None
        self.object_center_x = None
        self.green_detected = False
        self.distance_ahead = float('inf')

        # Subscribers & Publisher
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size=1)
        self.scan_sub  = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.opcv_pub   = rospy.Publisher('/opcv', Image, queue_size=1)


        self.rate = rospy.Rate(10)

    def image_cb(self, msg):
                
        self.opcv_pub.publish(msg)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn(f"CV bridge failed: {e}")
            return

        h, w = frame.shape[:2]
        self.image_width = w

        # Convert to HSV and threshold for green
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lower = np.array([38, 18, 18])
        # upper = np.array([86, 255, 255])
        lower = np.array([40, 30, 10])
        upper = np.array([75, 255, 255])

        mask  = cv2.inRange(hsv, lower, upper)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.green_detected = False
            return
        
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 1000:
            self.green_detected = False
            return

        # Fit a minimum-area rotated rectangle
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Compute lengths of its four sides
        def dist(a, b): return np.linalg.norm(a - b)
        edges = [(box[i], box[(i+1)%4], dist(box[i], box[(i+1)%4])) for i in range(4)]

        # Group edges into opposite pairs
        pairs = [edges[0:2], edges[2:4]]
        # Sum lengths in each pair → gives dimensions
        sum0 = pairs[0][0][2] + pairs[0][1][2]
        sum1 = pairs[1][0][2] + pairs[1][1][2]
        # Choose the longer side pair (big side)
        long_pair = pairs[0] if sum0 > sum1 else pairs[1]

        # Compute midpoint of that longer side
        p1, p2, _ = long_pair[0]
        mid = ((p1 + p2) / 2).astype(int)

        self.object_center_x = mid[0]
        self.green_detected = True



    def scan_cb(self, msg):
        # Define the frontal 30 degree window (±15 degrees)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        front_angle_min = -math.radians(15)
        front_angle_max = math.radians(15)

        # Compute index range
        index_min = int((front_angle_min - angle_min) / angle_increment)
        index_max = int((front_angle_max - angle_min) / angle_increment)
        index_min = 360 - abs(index_min)


        front_left_ranges = ranges[index_min:360]
        front_right_ranges = ranges[0:index_max+1]
        front_ranges = front_left_ranges + front_right_ranges

        # if index_min < index_max:
        #     front_ranges = ranges[index_min:index_max+1]
        # else:
        #     front_ranges = ranges[index_min:] + ranges[:index_max+1]

        # # Filter valid measurements
        # front_distances = [r for r in front_ranges if not math.isnan(r) and r > msg.range_min and r < msg.range_max]
        # if front_distances:
        #     self.distance_ahead = min(front_distances)
        # else:
        #     self.distance_ahead = float('inf')

    def run(self):
        rospy.loginfo("TurtleBot3 Green Can Follower (adapted) started.")
        while not rospy.is_shutdown():
            twist = Twist()

            if self.green_detected and self.image_width:
                # Steering: turn to center the long side's midpoint
                error = self.object_center_x - (self.image_width / 2)
                twist.angular.z = - self.turn_gain * error

                # Forward/backward to reach target distance
                if self.distance_ahead > self.target_distance + 0.1:
                    twist.linear.x = self.forward_speed
                elif self.distance_ahead < self.target_distance - 0.1:
                    twist.linear.x = -0.1  # back off if too close
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.loginfo("✅ Arrived at 1 m from trash can big side.")
            else:
                # Search behavior
                twist.linear.x  = 0.0
                twist.angular.z = self.search_turn_speed

            self.cmd_pub.publish(twist)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        robot = GarbageRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass