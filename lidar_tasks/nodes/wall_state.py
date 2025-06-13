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


ROS_WALL_DISTANCE_TOPIC: Final[str] = "/wall_distance"
ROS_WALL_MOTION_TOPIC: Final[str] = "/wall_motion"
ROS_MARKER_TOPIC: Final[str] = "/wall_motion_marker"
ROS_FILTERED_TOPIC: Final[str] = "/scan_filtered"

class WallStatePublisher:
    def __init__(self):
        rospy.init_node('wall_state_publishre')

        self.angle_width_deg = rospy.get_param('~angle_width_deg', 2.5)  
        self.stable_threshold = rospy.get_param('~stable_threshold', 0.03)  

        # Параметры
        self.prev_distances = []
        self.threshold = 0.02
        self.fov_degrees = 30  
        self.last_msg = LaserScan()

        self.pub_dist = rospy.Publisher(ROS_WALL_DISTANCE_TOPIC, Float32, queue_size=1)
        self.pub_motion = rospy.Publisher(ROS_WALL_MOTION_TOPIC, String, queue_size=1)
        self.pub_marker = rospy.Publisher(ROS_MARKER_TOPIC, Marker, queue_size=5)
            
        rospy.Subscriber(ROS_FILTERED_TOPIC, LaserScan, self.scan_cb)
            
        rospy.loginfo("WallStatePublisher")
        rospy.spin()

    def scan_cb(self, scan: LaserScan):
        center_indexes = len(scan.ranges) / 2
        left_indexes = int(center_indexes - self.angle_width_deg)
        right_indexes = int(center_indexes + self.angle_width_deg) + 1
        
        sector = scan.ranges[left_indexes:right_indexes]
        valid = [r for r in sector if not math.isnan(r) and not math.isinf(r) and r > 0]
        
        if not valid:
            rospy.logwarn_throttle(5, "No valid scans in sector." % self.angle_width_deg)
            return
        
        distance = min(valid)
        self.pub_dist.publish(Float32(distance))
        
        if len(self.prev_distances) < 4:
            state = "STABLE"
        else:
            diffs = [abs(self.prev_distances[i+1] - self.prev_distances[i]) for i in range(len(self.prev_distances)-1)]
            delta = sum(diffs)
            if abs(delta) <= self.stable_threshold:
                state = "STABLE"  +" delta " + str(abs(delta))
            elif self.prev_distances[-1] < self.prev_distances[0]:
                state = "APPROACHING" +" delta " + str(abs(delta))
            else:
                state = "RECEDING"  +" delta " + str(abs(delta))
        
        self.pub_motion.publish(String(state))
        self.prev_distances.append(distance)
        if len(self.prev_distances) > 5:
            self.prev_distances.pop(0)
        
        # Создание маркера
        marker = Marker()
        marker.header.frame_id = scan.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wall_motion"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        

        if state == "APPROACHING":
            yaw = 0.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 1.0)
        elif state == "RECEDING":
            yaw = math.pi
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 1.0)
        else:
            yaw = 0.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.0, 1.0, 1.0)
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        marker.scale.x = distance
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        self.pub_marker.publish(marker)

if __name__ == '__main__':
    try:
        node = WallStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass