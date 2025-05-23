import rospy
from sensor_msgs.msg import LaserScan
import math

class LidarTest:
    def __init__(self):
        rospy.init_node('frontal_lidar_reader')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.laser_pub = rospy.Publisher('/test_lidar', LaserScan, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        print(len(ranges))

        # Define the frontal 30 degree window (±15 degrees)
        front_angle_min = -math.radians(15)
        front_angle_max = math.radians(15)

        # Compute index range
        index_min = int((front_angle_min - angle_min) / angle_increment)
        index_max = int((front_angle_max - angle_min) / angle_increment)
        index_min = 360 - abs(index_min)
        print(index_min)
        print(index_max)

        # Extract frontal ranges
        front_ranges = ranges[index_min:index_max+1]
        avg = sum(ranges) / len(ranges)
        new_msg = msg
        new_msg.ranges = front_ranges

        # You can now use these values, e.g., find minimum distance in front
        front_distances = [r for r in front_ranges if not math.isnan(r) and r > 0]
        if front_distances:
            min_front = min(front_distances)
            print(f"Min distance in front 30°: {min_front:.2f} m {avg}")
        else:
            print(f"No valid measurements in front sector. {avg}")
        self.laser_pub.publish(new_msg)

    def run(self):
        rospy.loginfo("TurtleBot3 Green Can Follower started.")
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        robot = LidarTest()
        robot.run()
    except rospy.ROSInterruptException:
        pass


