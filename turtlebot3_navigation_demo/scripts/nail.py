#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import cv2
from cv_bridge import CvBridge

class mover:
    def __init__(self):
        rospy.init_node("trashbin_move")
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.speed = Twist()
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.image_pub = rospy.Publisher("/otladka", Image, queue_size=10)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)

        self.direction = 2
        self.move_time = 0.2
        self.turn_time = 0.2
        self.danger_distance = 0.5
        self.obstacle_front = False
        self.obstacle_right = False
        self.obstacle_left = False

        self.cx = 0
        self.cy = 0
        self.width = 0

        while not rospy.is_shutdown():
            self.move()
            self.rate.sleep()

    def image_callback(self, msg):
        image_rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
        H_min = 40
        H_max = 75
        S_min = 30
        S_max = 255
        V_min = 10
        V_max = 255
        hsv_min = (H_min, S_min, V_min)
        hsv_max = (H_max, S_max, V_max)
        image_mask = cv2.inRange(image_hsv, hsv_min, hsv_max)
        image_green = cv2.bitwise_and(image_rgb, image_rgb, mask=image_mask)
        image_gray = cv2.cvtColor(image_green, cv2.COLOR_RGB2GRAY)

        moments = cv2.moments(image_mask)
        if moments["m00"] != 0:
            self.cx = int(moments['m10'] / moments['m00'])
            self.cy = int(moments['m01'] / moments['m00'])
            half_width = image_rgb.shape[1] / 2
            
            
            cv2.circle(image_green, (self.cx, self.cy), 10, (255, 0, 0), -1)

            if abs(half_width - self.cx) < 30:
                self.direction = 0
                
            elif (half_width - self.cx) < 0:
                self.direction = 1
               
            elif (half_width - self.cx) > 0:
                self.direction = -1

            else:
                self.direction = 2
        else:
            self.cx = 0
            self.cy = 0

        messege = self.bridge.cv2_to_imgmsg(image_green, "rgb8")
        self.image_pub.publish(messege)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        if self.cx != 0 and self.cy != 0:
            print("CENTROID " + str(self.cx) + " " + str(self.cy))
            depth_value = depth_image[self.cy, self.cx]
            if depth_value == "nan":
                return
            
            if depth_value <= 1:
                self.obstacle_front = True
            else:
                self.obstacle_front = False
            print("dist to green: ", depth_value)

    def move(self):
        if self.obstacle_front == True:
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.speed_pub.publish(self.speed)
        
        elif self.obstacle_right == True:
            self.speed.angular.z = -1
            self.speed_pub.publish(self.speed)
            time.sleep(1.59)
            self.speed.angular.z = 0
            self.speed.linear.x = 0.2
            self.speed_pub.publish(self.speed)
            time.sleep(2)
            self.speed.linear.x = 0
            self.speed.angular.z = 1
            self.speed_pub.publish(self.speed)
            time.sleep(1.59)
            self.speed.angular.z = 0
            self.speed_pub.publish(self.speed)

        elif self.obstacle_left == True:
            self.speed.angular.z = 1
            self.speed_pub.publish(self.speed)
            time.sleep(1.59)
            self.speed.angular.z = 0
            self.speed.linear.x = 0.2
            self.speed_pub.publish(self.speed)
            time.sleep(2)
            self.speed.linear.x = 0
            self.speed.angular.z = -1
            self.speed_pub.publish(self.speed)
            time.sleep(1.59)
            self.speed.angular.z = 0
            self.speed_pub.publish(self.speed)

        elif self.direction == 0:
            self.speed.linear.x = 0.07
            self.speed_pub.publish(self.speed)
            time.sleep(self.move_time)
            self.speed.linear.x = 0
            self.speed_pub.publish(self.speed)

        elif self.direction == -1:
            self.speed.angular.z = 0.05
            self.speed_pub.publish(self.speed)
            time.sleep(self.turn_time)
            self.speed.angular.z = 0
            self.speed_pub.publish(self.speed)

        elif self.direction == 1:
            self.speed.angular.z = -0.05
            self.speed_pub.publish(self.speed)
            time.sleep(self.turn_time)
            self.speed.angular.z = 0
            self.speed_pub.publish(self.speed)
        
    def get_angle_indices(self, msg, angle_min, angle_max):
        indices = []
        angle_min_rad = angle_min * (3.14159265359 / 180)
        angle_max_rad = angle_max * (3.14159265359 / 180)
        
        for i in range(len(msg.ranges)):
            current_angle = msg.angle_min + i * msg.angle_increment
            if angle_min_rad <= current_angle <= angle_max_rad:
                indices.append(i)
        return indices
    
    def laser_callback(self, msg):
        self.obstacle_right = False
        self.obstacle_left = False

        right_start = 30  
        right_end = 35
        right_indices = self.get_angle_indices(msg, right_start, right_end)
        for idx in right_indices:
            distance = msg.ranges[idx]
            if 0 < distance < self.danger_distance:
                self.obstacle_right = True
                break
        
        left_start = 325
        left_end = 330
        left_indices = self.get_angle_indices(msg, left_start, left_end)
        for idx in left_indices:
            distance = msg.ranges[idx]
            if 0 < distance < self.danger_distance:
                self.obstacle_left = True
                break

    def __del__(self):
        self.speed.angular.z = 0
        self.speed.linear.x = 0
        self.speed_pub.publish(self.speed)
    
def main():
    go_pridurok = mover()

if __name__ == '__main__':
    main()