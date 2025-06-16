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


ROS_WALL_MOTION_TOPIC: Final[str] = "/obstacle_movement"
ROS_SCAN_TOPIC: Final[str] = "/scan2"

class LidarStatePublisher:
    def __init__(self):
        rospy.init_node('LiDAR_state_publishre')

        # Параметры
        self.prev_distances = []
        self.threshold = 0.02
        self.last_msg = LaserScan()

        self.pub_dist = rospy.Publisher(ROS_WALL_MOTION_TOPIC, String, queue_size=1)
            
        rospy.Subscriber(ROS_SCAN_TOPIC, LaserScan, self.scan_cb)
        rospy.loginfo("LidarStatePublisher")
        rospy.spin()

    def scan_cb(self, scan: LaserScan):
        print(12)

        spread = math.radians(15) / scan.angle_increment
        center = int((0 - scan.angle_min) / scan.angle_increment)
        minIndex = int(max(0, center - spread))
        maxIndex = int(min(len(scan.ranges) - 1, center + spread))

        
        sector = scan.ranges[minIndex:maxIndex]
        valid = [r for r in sector if not math.isnan(r) and not math.isinf(r) and r > 0]


        distance = min(valid)
        
        if not valid:
            rospy.logwarn_throttle(5, "No valid scans in sector." % self.angle_width_deg)
            return
             
        if len(self.prev_distances) < 4:
            state = "STABLE"
        else:
            diffs = [abs(self.prev_distances[i+1] - self.prev_distances[i]) for i in range(len(self.prev_distances)-1)]
            delta = sum(diffs)
            if abs(delta) <= self.threshold:
                state = "STABLE"  +" delta " + str(abs(delta))
            elif self.prev_distances[-1] < self.prev_distances[0]:
                state = "APPROACHING" +" delta " + str(abs(delta))
            else:
                state = "RECEDING"  +" delta " + str(abs(delta))
        
        self.pub_dist.publish(String(state))
        self.prev_distances.append(distance)
        if len(self.prev_distances) > 5:
            self.prev_distances.pop(0)     

if __name__ == '__main__':
    try:
        node = LidarStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass