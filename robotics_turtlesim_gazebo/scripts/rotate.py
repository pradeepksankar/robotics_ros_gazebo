#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
	rospy.init_node('robot_cleaner', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	
	print("Let's rotate your robot")
	speed = 30 
	angle = random.randint(0, 360) 
	clockwise = 0
	
	angular_speed = speed*2*PI/360
	relative_angle = angle*2*PI/360

	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

	if clockwise:
		vel_msg.angular.z = -abs(angular_speed)
	else:
		vel_msg.angular.z = abs(angular_speed)

	t0 = rospy.Time.now().to_sec()
	current_angle = 0

	while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)

	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	rospy.spin()

if __name__ == '__main__':
	try:
		rotate()
	except rospy.ROSInterruptException:
		pass
