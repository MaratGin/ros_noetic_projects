import numpy as np
import math
import statistics
from typing import List, Optional, Tuple
from scipy.signal import medfilt

import rospy
from sensor_msgs.msg import LaserScan
from typing import Final
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import time


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits

        self.last_error = 0.0
        self.last_time = None

    def reset(self):
        self.last_error = 0.0
        self.last_time = None

    def compute(self, error):
        now = time.time()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time
        self.last_time = now

        # Proportional
        p = self.kp * error

        # Derivative
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        d = self.kd * derivative


        self.last_error = error
        output = p + d

        # Если скорость выходит за обозначенные границы
        if self.max_out is not None:
            output = min(output, self.max_out)
        if self.min_out is not None:
            output = max(output, self.min_out)
        print(output)

        return output


ROS_WALL_DISTANCE_TOPIC: Final[str] = "/wall_distance"
ROS_FILTERED_TOPIC: Final[str] = "/scan_filtered"

class PIDRobot:
    def __init__(self):
        rospy.init_node('pid_robot_node')

        self.target_distance = rospy.get_param('~target_distance', 1.0)       # meters
        self.angle_width_deg = rospy.get_param('~angle_width_deg', 2.5)  

        kp = rospy.get_param('~kp', 0.3)
        ki = rospy.get_param('~ki', 0.0)
        kd = rospy.get_param('~kd', 1.6)
        max_speed = rospy.get_param('~max_speed', 2.8) 

        self.controller = PIDController(kp, ki, kd, output_limits=(-max_speed, max_speed))
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.current_distance = None


        self.sub_dist = rospy.Subscriber(ROS_WALL_DISTANCE_TOPIC, Float32, self.dist_cb, queue_size=1)
        rospy.Subscriber(ROS_FILTERED_TOPIC, LaserScan, self.scan_cb)

        # Таймер для подсчет пид контроллера
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
      
        rospy.loginfo("PIDRobot")
        rospy.spin()

    def dist_cb(self, msg):
        self.current_distance = msg.data

    def scan_cb(self, scan: LaserScan):
        center_indexes = len(scan.ranges) / 2
        left_indexes = int(center_indexes - self.angle_width_deg)
        right_indexes = int(center_indexes + self.angle_width_deg) + 1
        
        sector = scan.ranges[left_indexes:right_indexes]
        valid = [r for r in sector if not math.isnan(r) and not math.isinf(r) and r > 0]
        
        if not valid:
            rospy.logwarn_throttle(5, "No valid scans in sector.")
            return
        
        distance = min(valid)
        self.current_distance = distance


    def control_loop(self, event):
        if self.current_distance is None:
            return

        # Подсчет отклонения от целевого расстояния
        error = self.target_distance - self.current_distance

        # Подсчет значения пид контроллера
        control = self.controller.compute(error)

        cmd = Twist()
        cmd.linear.x = -control
        cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)


if __name__ == '__main__':
    try:
        node = PIDRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass