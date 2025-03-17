#! /usr/bin/env python

import rospy

import actionlib

import math

import simple_counter.msg


class CounterAction(object):
	# create messages that are used to publish feedback/result
	_feedback = simple_counter.msg.CounterFeedback()
	_result = simple_counter.msg.CounterResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, simple_counter.msg.CounterAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	

	def execute_cb(self, goal):
		rospy.loginfo(goal.start)
		rospy.loginfo(goal.end)

		def get_nth_primes(start: int, end: int):
			rospy.loginfo('%s: START PRIME')
			count  = 1
			def is_prime(n):
				if n < 2:
					return False
				for i in range(2, int(n ** 0.5) + 1):
					if n % i == 0:
						return False
				return True
			primes = []
			num = 2
			while count <= end:
				if is_prime(num):
					print("NEW PRIME " + str(num))
					if (count >=start):
						print("COUNT" + str(count) + "START" + str(start))
						primes.append(num)
					count += 1
				num += 1
			rospy.loginfo('%s: END PRIME')
			return primes


		# helper variables
		r = rospy.Rate(0.5)
		success = True
        
		# append the seeds for the fibonacci sequence
		self._feedback.sequence = []
		self._feedback.sequence.append(0)
		self._feedback.sequence.append(1)

		# publish info to the console for the user
		rospy.loginfo('%s: Executing, creating  sequence of prime numbers')

		primes = get_nth_primes(goal.start, goal.end)
		rospy.loginfo(primes)

		# start executing the action
		for prime in primes:
			# check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self._action_name)
				self._as.set_preempted()
				success = False
				break
			self._feedback.sequence.append(prime)
			# publish the feedback
			self._as.publish_feedback(self._feedback)
			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if success:
			self._result.sequence = self._feedback.sequence
			rospy.loginfo('%s: Succeeded' % self._action_name)
			self._as.set_succeeded(self._result)
			
	

if __name__ == '__main__':
	rospy.init_node('counter')
	server = CounterAction(rospy.get_name())
	rospy.spin()
