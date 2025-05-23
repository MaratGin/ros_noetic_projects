#!/usr/bin/env python3
import rospy
import cv2
import math
import time
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class GarbageRobot:
    def __init__(self):
        rospy.init_node('green_can_follower_final')

        # Parameters
        self.target_distance =  1.0  
        self.forward_speed  = rospy.get_param('~forward_speed', 0.1)     # m/s
        self.turn_gain = 0.4      
        self.search_turn_speed = rospy.get_param('~search_turn_speed', 0.1)

        # State
        self.bridge = CvBridge()
        self.image_width = None
        self.object_center_x = None
        self.green_detected = False
        self.distance_ahead = float('inf')
        self.min_dist_obstacle = 0.5

        self.obstacle_right = False
        self.obstacle_left = False

        self.centroid_x = None  
        self.centroid_y = None  
        self.distance = float("inf")   
        
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
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

        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        
        lower = np.array([38, 18, 18])
        upper = np.array([86, 255, 255])

        mask  = cv2.inRange(hsv, lower, upper)
        print("ORIG IMAGE " + str(frame.shape))


        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.green_detected = False
            return

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 1000:
            self.green_detected = False
            return

        # Compute centroid of the contour
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            self.centroid_x = int(M['m10'] / M['m00'])
            self.centroid_y = int(M['m01'] / M['m00'])
        
            # Draw centroid (for visualization)
            cv2.circle(frame, (self.centroid_x, self.centroid_y), 5, (0, 255, 0), -1)
            print("GREEN DETECTED")
            self.green_detected = True

            print("CENTROID: " + str(self.centroid_x)+ " : " + str(self.centroid_y))    





    def scan_cb(self, msg):
        # Define the frontal 30 degree window (±15 degrees)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        self.obstacle_right = False
        self.obstacle_left = False
        right_angle = 90
        left_angle = 180
        self.obstacle_left = self.check_side(left_angle,15,msg)
        self.obstacle_right = self.check_side(right_angle,15,msg)
        print("-------- OBSTACLE CHECK "+ str(self.obstacle_left)+ " | " + str(self.obstacle_right)+ "---------")

    
    def check_side(self, angle, spread, scan):
        left = self.to_rad((angle - spread))
        right = self.to_rad((angle + spread))
        isObstacleInSide = False
        minIndex = math.ceil((left-scan.angle_min) / scan.angle_increment)
        maxIndex = math.floor((right - scan.angle_min) / scan.angle_increment)
        average = 0


        currIndex = minIndex + 1
        while currIndex <= maxIndex:
            if scan.ranges[currIndex] == float('nan'):
                average += scan.range_max
            else:
                average += scan.ranges[currIndex]
            if scan.ranges[int(currIndex)] < self.min_dist_obstacle:
                isObstacleInSide = True
                break
            currIndex = currIndex + 1
            avg = average / (maxIndex - minIndex)
            
        return isObstacleInSide

    def to_rad(self, degree):
        return math.radians(degree)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        # print("DEPTH IMAGE " + str(depth_image.shape))

        if self.centroid_x != 0 and self.centroid_y != 0:
            depth_value = depth_image[self.centroid_y, self.centroid_x]
            # print("------- VAL= " + str(type(depth_value)) + " |||| " + str(depth_value))
            if isinstance(depth_value, np.ndarray) or depth_value == float("nan") or depth_value == "nan" or math.isnan(depth_value):
                depth_value = float("inf")
                self.distance = depth_value
                return
            self.distance = depth_value
            # if depth_value <= 1:
                # self.obstacle_front = True
            # else:
                # self.obstacle_front = False
            print("dist to Green: ", depth_value)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            twist = Twist()
            self.obstacle_manager()
            # print("IS CENTR " + str(self.centroid_x) + " " + str(self.centroid_y))
            if self.centroid_x is not None and self.distance is not None:
                
                # print("DEPTH IN MOVING " + str(self.distance))
                if self.distance > self.target_distance and self.distance != float("inf"):
                    
                    error_x = self.centroid_x - 960 
                    
            
                    twist.angular.z = -float(error_x) / 1500  
                    twist.linear.x = self.forward_speed
                elif self.distance != float("inf"):
                    # Stop the robot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.loginfo("Target reached!")
                else: 
                    twist.angular.z = self.turn_gain

            else:
                twist.angular.z = self.turn_gain
            
            self.cmd_pub.publish(twist)
            rate.sleep()
    
    def obstacle_manager(self):
        if self.obstacle_right:
            self.avoid_obstacle(-1)
        elif self.obstacle_left:
            self.avoid_obstacle(1)
    
    def avoid_obstacle(self, multiplier):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

        twist.angular.z = 0.7 * multiplier
        self.cmd_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0
        twist.linear.x = 0.25
        self.cmd_pub.publish(twist)
        time.sleep(1)
        twist.linear.x = 0
        twist.angular.z = -0.7 * multiplier
        self.cmd_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)



if __name__ == '__main__':
    try:
        robot = GarbageRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass