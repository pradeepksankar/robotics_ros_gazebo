#!/usr/bin/env python
import random
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.msg import *
from turtlesim.srv import *

class TurtleController():

	def __init__(self):
		rospy.init_node('turtlebot_controller', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_Callback)
		rospy.wait_for_service("/turtle1/set_pen")
		setPen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
		setPen(255, 0, 0, 3, 0)

		self.pose = Pose()
		self.rate = rospy.Rate(1)
		self.vel_msg = Twist()

		while not rospy.is_shutdown():
			self.move_spiral()

	def pose_Callback(self, data):
		self.pose = data

	def move_spiral(self):
		constant_speed = 0 
		linear_x = 0.25 
		delta = 0 

		rospy.sleep(10)

		while((self.pose.x < 10.5) and (self.pose.y < 10.5)):
			linear_x = linear_x + delta
			self.vel_msg.linear.x = linear_x
			self.vel_msg.linear.y = 0
			self.vel_msg.linear.z = 0

			self.vel_msg.angular.x = 0
			self.vel_msg.angular.y = 0
			self.vel_msg.angular.z = constant_speed

			self.velocity_publisher.publish(self.vel_msg)

			self.rate.sleep()

		self.vel_msg.linear.x = 0 
		self.vel_msg.angular.z = 0 
		self.velocity_publisher.publish(self.vel_msg)

		print("Finished !!")

		rospy.spin()

if __name__ == "__main__":

	TurtleController()	
