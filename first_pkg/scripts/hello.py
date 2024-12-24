#!/usr/bin/env python
import rospy

def hello() :
	rospy.init_node('hello', anonymous=False)
	rate = rospy.Rate(2) # 10hz
	count = 0
	while not rospy.is_shutdown():
		hello_str = "hello world -- %s" % count
		count = count + 1
		rospy.loginfo(hello_str)
		rate.sleep()

if __name__ == "__main__":
	try:
		hello()
	except rospy.ROSInterruptException: 
		pass
