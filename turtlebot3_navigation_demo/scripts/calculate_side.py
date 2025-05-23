#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SideClassifier:
    def __init__(self):
        rospy.init_node('side_classifier', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        
        # Initialize variables
        self.centroid = None            # (x, y) of detected side's centroid
        self.depth = None               # Depth at centroid (meters)
        self.latest_rgb = None          # Latest RGB image
        self.latest_depth = None        # Latest depth image
        self.side_type = "unknown"      # "smaller" or "bigger"
        self.offset_px = 0.0            # Centroid offset from image center (pixels)
        
        # HSV range for green
        self.lower_green = np.array([38, 18, 18])
        self.upper_green = np.array([86, 255, 255])
        
        # Camera intrinsics (1920x1080 resolution, adjust if needed)
        self.fx = 1386.36   # Focal length x (pixels)
        self.fy = 1385.06   # Focal length y (pixels)
        self.cx = 960    # Image center x
        self.cy = 540    # Image center y

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.detect_side()

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def detect_side(self):
        if self.latest_rgb is None:
            return
        
        hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get rotated bounding box
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Calculate aspect ratio of the bounding box
            width_px = max(rect[1][0], rect[1][1])  # Larger side
            height_px = min(rect[1][0], rect[1][1])  # Smaller side
            aspect_ratio = width_px / height_px
            
            # Classify side based on aspect ratio
            if 0.9 < aspect_ratio < 1.1:
                self.side_type = "smaller"  # Square (1m×1m)
            elif 1.9 < aspect_ratio < 2.1:
                self.side_type = "bigger"   # Rectangle (2m×1m)
            else:
                self.side_type = "unknown"
            
            # Calculate centroid
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                self.centroid = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                
                # Compute offset from image center (in pixels)
                self.offset_px = np.sqrt(
                    (self.centroid[0] - self.cx)**2 + 
                    (self.centroid[1] - self.cy)**2
                )
                
                # Get depth at centroid
                if self.latest_depth is not None:
                    x, y = int(self.centroid[0]), int(self.centroid[1])
                    self.depth = self.latest_depth[y, x]
                    
                    if not np.isnan(self.depth) and not np.isinf(self.depth):
                        # Verify real-world dimensions (optional)
                        self.calculate_real_dimensions(width_px, height_px)
            
            # Draw results (for visualization)
            cv2.drawContours(self.latest_rgb, [box], 0, (0, 255, 0), 2)
            cv2.putText(self.latest_rgb, f"Side: {self.side_type}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Detection', self.latest_rgb)
            cv2.waitKey(1)

    def calculate_real_dimensions(self, width_px, height_px):
        # Convert pixel dimensions to meters using depth
        width_m = (width_px * self.depth) / self.fx
        height_m = (height_px * self.depth) / self.fy
        
        # Log results (optional)
        rospy.loginfo(f"Classified: {self.side_type} | Offset: {self.offset_px:.1f} px | "
                      f"Real Dimensions: {width_m:.2f}m x {height_m:.2f}m")

if __name__ == '__main__':
    try:
        classifier = SideClassifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass