#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math
from turtlesim.srv import TeleportAbsolute, SetPen


def pose_callback(pose):
    rospy.loginfo("Robot X=%f : Y=%f : Z=%f\n", pose.x, pose.y, pose.theta)
 
# сервис для мгновенной телепортации черепашки
def teleport_to(x,y,z):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        teleport(x,y,z)
    except rospy.ServiceException as e:
        print("Service fail %s"%e) 

# сервис выключения рисования пути черепашки
def pen_off():
    rospy.wait_for_service('/turtle1/set_pen')
    try:
        pen_off = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        #       r   g   b   w    flag
        pen_off(210, 34, 45, 16, 255)
    except rospy.ServiceException as e:
        print("Pen service fail %s"%e)

# сервис включения рисования пути черепашки
def pen_on():
    rospy.wait_for_service('/turtle1/set_pen')
    try:
        pen_off = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        #       r  g  b   w  flag
        pen_off(210,34,45, 16, 0)
    except rospy.ServiceException as e:
        print("Pen service fail %s"%e)


def move_turtle(lin_vel,ang_vel):

    FORWARD_SPEED = 2

    rospy.init_node('draw_letter', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)

    rate = rospy.Rate(10) # 10hz
    angle = 2*math.pi*90/360 + 0.1
    distance = 6
    horizontal_distance = 3
    angle_speed = 30*2*math.pi/360
    vel = Twist()
    pen_off()
    teleport_to(3,4,0)
    pen_on()
    vel.linear.x = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    # vel.angular.z = abs(angle_speed)
    vel.angular.z = 0
    vel.linear.y = FORWARD_SPEED
    rospy.loginfo("LinearVel=%f: AngularVel=%f", lin_vel, ang_vel)
    while not rospy.is_shutdown():
        print("Start")
        # rotate 90 degrees
        # cur_angle = 0
        # t0 = rospy.Time().now().to_sec()
        # while(cur_angle < angle):
        #     t1=rospy.Time().now().to_sec()
        #     cur_angle = abs((t1-t0)*vel.angular.z)
        #     pub.publish(vel)
        # vel.linear.x = 2
        # vel.angular.z = 0

        # Левая сторона Н
        cur_distance = 0
        t0 = rospy.Time().now().to_sec()
        while(cur_distance < distance):
            t1 = rospy.Time().now().to_sec()
            cur_distance = abs((t1-t0)*vel.linear.y)
            pub.publish(vel)

        total = cur_distance
        cur_distance = 0
        vel.linear.y = -2
        
        # Возвращение к центру линии
        t0 = rospy.Time().now().to_sec()
        while(cur_distance < total/2):
            t1 = rospy.Time().now().to_sec()
            cur_distance = abs((t1-t0)*vel.linear.y)
            pub.publish(vel)
        
        vel.linear.y = 0
        vel.linear.x = 2
        
        # Рисовка второго элемента Н
        cur_distance = 0
        t0 = rospy.Time().now().to_sec()
        while(cur_distance < horizontal_distance):
            t1 = rospy.Time().now().to_sec()
            cur_distance = abs((t1-t0)*vel.linear.x)
            pub.publish(vel)
        
        vel.linear.x = 0
        vel.linear.y = FORWARD_SPEED
        
        cur_distance = 0
        # Рисовка верхней половины правой линии Н
        t0 = rospy.Time().now().to_sec()
        while(cur_distance < distance/2):
            t1 = rospy.Time().now().to_sec()
            cur_distance = abs((t1-t0)*vel.linear.y)
            pub.publish(vel)

        total = cur_distance
        cur_distance = 0
        vel.linear.x = 0
        vel.linear.y = -FORWARD_SPEED
        
        # Риовка оставшейся нижней части
        t0 = rospy.Time().now().to_sec()
        while(cur_distance < distance):
            t1 = rospy.Time().now().to_sec()
            cur_distance = abs((t1-t0)*vel.linear.y)
            pub.publish(vel)
        
        vel.linear.y = 0

        
        # vel.angular.z = 1
        # while True:
        #     vel.angular.z = 1

        vel.linear.y = 0
        pub.publish(vel)
        # rate.sleep()

if __name__ == '__main__':
    if True:
        print("Hello")
    try:
        move_turtle(float(sys.argv[1]),float(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass