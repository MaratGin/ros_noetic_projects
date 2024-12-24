#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import math
import enum

class Sensor:
	MIN_DIST_FROM_OBSTACLE = 0.4
	def __init__(self, angle, spread):
		self.left = self.to_rad((angle - spread))
		self.right = self.to_rad((angle + spread))

	def is_free(self, scan: LaserScan):
		isObstacleInFront = False
		minIndex = math.ceil((self.left-scan.angle_min) / scan.angle_increment)
		maxIndex = math.floor((self.right - scan.angle_min) / scan.angle_increment)
		average = 0

		currIndex = minIndex + 1
		while currIndex <= maxIndex:
			if scan.ranges[currIndex] == float('nan'):
				average += scan.range_max
			else:
				average += scan.ranges[currIndex]

			if scan.ranges[int(currIndex)] < self.MIN_DIST_FROM_OBSTACLE:
				isObstacleInFront = True
				break
			currIndex = currIndex + 1
		return isObstacleInFront, average / (maxIndex - minIndex)

	def to_rad(self, degree):
		return math.radians(degree)




class RobotState(enum.Enum):
    forward = 1
    turn_left = 2
    forward_left = 3
    find_wall = 4


class RobotData:
	MIN_DIST_FROM_OBSTACLE = 0.4
	def __init__(self):
		self.state = RobotState.find_wall
		self.right = Sensor(90,15)
		self.front = Sensor(180,15)


	def set_left(angle, spread):
		pass

	def next_state(self):
		current_index = self.statuses.index(self.state)
		next_index = (current_index + 1) % len(self.statuses)
		self.state = self.statuses[next_index]

	
	def to_move(self):
		return self.state == RobotState.up or self.state == RobotState.down 
	
	def get_turn_angle(self):
		if self.state == RobotState.turn_left:
			return math.radians(-90)
		
		if self.state == RobotState.turn_right:
			return math.radians(90)
		


class WallerRobot:
	FORWARD_SPEED = 0.1
	MIN_SCAN_ANGLE = math.pi - (15.0/180.0*math.pi)
	MAX_SCAN_ANGLE = math.pi + (15.0/180.0*math.pi)
	MIN_DIST_FROM_OBSTACLE = 0.4
	ROTATE_SPEED = (10 * math.pi) / 180 
	keepMoving = True
	HORIZONTAL_DISTANCE = 0.2
	Kp = 2
	Kd = 0.4
	error_prev = 0

	def __init__(self):
		self.data = RobotData()
		rospy.init_node('stopper', anonymous=False)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
		self.ROTATE_SPEED = math.radians(30)
		self.right = 0
		self.front = 0
		self.right_avg = 0
		self.front_avg = 0

	def move_diagonal(self):
		rate = rospy.Rate(10) # 10hz
		vel = Twist()
		vel.linear.x = (-1)*self.FORWARD_SPEED
		vel.linear.y = (4)*self.FORWARD_SPEED
		self.pub.publish(vel)
		rate.sleep()

	
	def move_forward(self):
		vel = Twist()
		vel.angular.z = 0
		vel.linear.x = (-1)*self.FORWARD_SPEED
		self.pub.publish(vel)
		# if self.state.state == RobotStatus.down:
		# 	vel.linear.x = (-1)*self.FORWARD_SPEED
		# 	self.pub.publish(vel)
		# elif self.state.state == RobotStatus.up:
		# 	vel.linear.x = (1)*self.FORWARD_SPEED
		# 	self.pub.publish(vel)

	
	def start_move(self):
		# while not rospy.is_shutdown():
		# 	rospy.spin()
		rate = rospy.Rate(10) # 10hz
		twist = Twist()
		self.keepMoving = True
		while not rospy.is_shutdown():
			if (self.data.state == RobotState.find_wall):
				# print("NO WALL! FIND WALL")
					self.move_forward()
			elif (self.data.state == RobotState.turn_left):
					self.rotate90()
			elif (self.data.state == RobotState.forward):
				print(self.right_avg)
			# print(right_avg, front_avg)
				if (self.front_avg > 0.4):
				# print("MORE 0.4")
					twist.angular.z = 0
					twist.linear.x = -1*self.FORWARD_SPEED
					self.pub.publish(twist)
				else:
					print("LESS 0.4")
					twist.linear.x = 0
					self.pub.publish(twist)
					self.rotate90()
				if (self.right_avg > 0.42):
					if self.right_avg > 0.6:
						twist.angular.z = -4 * self.ROTATE_SPEED / 3
						twist.linear.x = 0
						self.pub.publish(twist)
					print("MORE 0.4")
					twist.angular.z = -1 * self.ROTATE_SPEED / 3
					twist.linear.x = 0
					self.pub.publish(twist)
				elif (self.right_avg < 0.38):
					print("LESS 0.4")
					twist.linear.x = 0
					twist.angular.z = 1 * self.ROTATE_SPEED / 3
					self.pub.publish(twist)

			# if self.state.to_move():
			# 	self.move()
			# 	rate.sleep()
			# else:
			# 	vel.linear.x = 0
			# 	self.rotate()
			# 	# direction = 1 if self.state.state == RobotStatus.turn_left else -1
			# 	direction = -1
			# 	self.move_horizontally(direction)
			# 	self.rotate()
			# 	self.state.next_state()
			# 	# self.pub.publish(vel)
			# 	rate.sleep()
	def get_front_left(self, scan: LaserScan):
		minIndex = 43
		maxIndex = 47
		currIndex = minIndex
		average = 0
		while currIndex <= maxIndex:
			if scan.ranges[currIndex] == float('nan'):
				average += scan.range_max
			else:
				average += scan.ranges[currIndex]
			currIndex += 1
		return (average / (maxIndex - minIndex))


		

	def rotate90(self):
		print("ROTATING")
		twist = Twist()
		angle = math.radians(94)
		twist.angular.z = abs(angle)
		cur_angle = 0
		t0 = rospy.Time().now().to_sec()
		while(cur_angle < angle):
			t1=rospy.Time().now().to_sec()
			cur_angle = abs((t1-t0)*twist.angular.z)
			self.pub.publish(twist)
        # vel.angular.z = 0
		# twist = Twist()
		# print("Rotating")
		# turn_angle = math.radians(90)

		# start = rospy.Time.now().to_sec()
		# end = start + rospy.Duration(abs(turn_angle) / self.ROTATE_SPEED).to_sec()
		# while rospy.Time.now().to_sec() < end:
		# 	twist.angular.z = 1 * self.ROTATE_SPEED
		# 	self.pub.publish(twist)
		twist.angular.z = 0
		self.pub.publish(twist)
		self.data.state = RobotState.forward
		print("END ROTATION")

	
	def callback(self, msg):
		twist = Twist()
		rate = rospy.Rate(10) # 10hz
		# isObstacleInFront = False
		# minIndex = round((self.MIN_SCAN_ANGLE - msg.angle_min) / msg.angle_increment)
		# maxIndex = round((self.MAX_SCAN_ANGLE - msg.angle_min) / msg.angle_increment)
		# print(minIndex, maxIndex, self.MIN_SCAN_ANGLE, self.MAX_SCAN_ANGLE, msg.angle_min, msg.angle_max)
		# print(len(msg.ranges))
		obstacle_right, right_avg = self.data.right.is_free(msg)
		obstacle_front, front_avg = self.data.front.is_free(msg)


		if self.data.state == RobotState.find_wall:
			if obstacle_front:
				self.data.state = RobotState.turn_left
				# print("NO WALL! FIND WALL"
		self.right = obstacle_right
		self.front = obstacle_front
		self.front_avg = front_avg
		self.right_avg = right_avg
			
		
		
		
		# elif self.data.state == RobotState.forward:
		# 	# print(right_avg, front_avg)
		# 	if (front_avg > 0.4):
		# 		# print("MORE 0.4")
		# 		twist.angular.z = 0
		# 		twist.linear.x = -1*self.FORWARD_SPEED
		# 		self.pub.publish(twist)

		# 	else:
		# 		print("LESS 0.4")
		# 		twist.linear.x = 0
		# 		self.pub.publish(twist)
		# 		self.rotate90()
				
			
		# elif (obstacle_right and obstacle_front):
		# 	print("LEFT AND FRONT OBSTACLE! TURNING")
		# 	self.turn_left()

		# elif (obstacle_front):
		# 	print("FRONT OBSTACLE! TURNING")
		# 	self.move_forward()
	
		# elif (obstacle_right):
		# 	print("RIGHT OBSTACLE! TURN LEFT")
		# 	self.turn_left()

		# elif (self.data.state == RobotState.find_wall):
		# 	print("NO WALL! FIND WALL")
		# 	self.move_forward()
		# else:
		# 	print("ALL FREE! MOVE RIGHT")
		# 	self.move_forward()

			# self.turn_right()		

		# if self.state.to_move():
		# 	pass
		# else:
		# 	self.keepMoving = False
		# 	self.state.next_state()

	def to_rad(degree):
		return math.radians(degree)

	def turn_left(self):
		rate = rospy.Rate(10) # 10hz
		twist = Twist()
		print("Turning")
		twist.angular.z = 1 * self.ROTATE_SPEED
		self.pub.publish(twist)
		rate.sleep()

	def turn_right(self):
		rate = rospy.Rate(10) # 10hz
		twist = Twist()
		print("Turning")
		twist.angular.z = -1 * self.ROTATE_SPEED
		self.pub.publish(twist)
		rate.sleep()		

	# def rotate(self):
	# 	twist = Twist()
	# 	print("Rotating")
	# 	turn_angle = self.state.get_turn_angle()

	# 	start = rospy.Time.now()
	# 	end = start + rospy.Duration(abs(turn_angle) / self.ROTATE_SPEED)
	# 	print(self.state.state)
	# 	while rospy.Time.now() < end:
	# 		twist.angular.z = (1 if self.state.state == RobotStatus.turn_left else -1) * self.ROTATE_SPEED
	# 		self.pub.publish(twist)
	# 	print("Stop Rotating")
	# 	twist.angular.z = 0
	# 	self.pub.publish(twist)
	# 	# self.state.next_state
	# 	# self.keepMoving = True
			

if __name__ == '__main__':
	try:
		st = WallerRobot()
		st.start_move()
	except rospy.ROSInterruptException:
		pass
