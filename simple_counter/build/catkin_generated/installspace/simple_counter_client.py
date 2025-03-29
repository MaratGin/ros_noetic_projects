#!/usr/bin/env python3

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib
import sys
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import simple_counter.msg

def simple_counter_client(start, end):
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('counter', simple_counter.msg.CounterAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = simple_counter.msg.CounterGoal(start=start, end=end)

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('simple_counter_client_py')
		if len(sys.argv) < 3:
			print("Not enough arguments!")
			sys.exit(1)
		start = int(sys.argv[1])
		end = int(sys.argv[2])

		result = simple_counter_client(start=start,end=end)
		print("Result:", ', '.join([str(n) for n in result.sequence]))
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
