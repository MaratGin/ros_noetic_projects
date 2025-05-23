#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

class GarbageCanDetector:
    def __init__(self):
        rospy.init_node('garbage_can_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize variables
        self.centroid_x = None        # X-coordinate of detected object
        self.centroid_y = None        # Y-coordinate (for depth lookup)
        self.distance = None          # Depth at centroid
        self.can_angle = None         # Orientation angle of the can
        self.robot_yaw = 0.0          # Robot's current yaw (from odometry)
        self.latest_depth_image = None # Store latest depth image
        self.state = "APPROACH"       # Control state: APPROACH or ALIGN
        
        # HSV range for green (tune as needed)
        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])
        
        # Control parameters
        self.target_distance = 1.0    # Stop at 1 meter
        self.linear_speed = 0.2       # Base forward speed
        self.angular_speed = 0.5      # Base turning speed
        self.angular_gain = 0.01      # PID gain for angle adjustment

    def rgb_callback(self, msg):
        # Process RGB image to detect green can
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Compute centroid
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                self.centroid_x = int(M['m10'] / M['m00'])
                self.centroid_y = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (self.centroid_x, self.centroid_y), 5, (0, 255, 0), -1)
            
            # Compute can orientation using minAreaRect
            rect = cv2.minAreaRect(largest_contour)
            self.can_angle = rect[2]  # Angle in [-90°, 0°]
        
        cv2.imshow('RGB View', cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        # Store latest depth image
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def odom_callback(self, msg):
        # Get robot's yaw from odometry
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.robot_yaw = np.degrees(yaw)  # Convert to degrees

    def get_depth_at_centroid(self):
        # Get depth value at centroid (if valid)
        if self.latest_depth_image is not None and self.centroid_x is not None and self.centroid_y is not None:
            depth = self.latest_depth_image[self.centroid_y, self.centroid_x]
            
            # Handle NaN/Inf values (common in depth images)
            if not np.isnan(depth) and not np.isinf(depth):
                self.distance = depth
            else:
                self.distance = None
        else:
            self.distance = None

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_depth_at_centroid()  # Update distance from depth image
            twist = Twist()
            
            if self.centroid_x is not None and self.distance is not None and self.can_angle is not None:
                if self.state == "APPROACH":
                    # Move toward the can until 1m away
                    if self.distance > self.target_distance:
                        # Center the can horizontally
                        error_x = self.centroid_x - 960  # 640x480 image
                        twist.angular.z = -float(error_x) * 0.001  # Tune this gain
                        twist.linear.x = self.linear_speed
                    else:
                        # Stop and switch to ALIGN state
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.state = "ALIGN"
                        rospy.loginfo("Switching to ALIGN state")
                
                elif self.state == "ALIGN":
                    # Calculate desired angle (perpendicular to can's orientation)
                    desired_angle = self.can_angle + 90  # 90° offset
                    
                    # Normalize angles to [-180°, 180°]
                    current_angle = self.robot_yaw
                    error_angle = (desired_angle - current_angle + 180) % 360 - 180
                    
                    # Rotate to align
                    if abs(error_angle) > 5:  # 5° tolerance
                        twist.angular.z = self.angular_gain * error_angle
                    else:
                        twist.angular.z = 0.0
                        rospy.loginfo("Aligned perpendicularly!")
            
            else:
                # Search for the can (rotate slowly)
                twist.angular.z = self.angular_speed
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = GarbageCanDetector()
        detector.control_loop()
    except rospy.ROSInterruptException:
        pass