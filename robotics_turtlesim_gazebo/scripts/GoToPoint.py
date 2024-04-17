#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

# A partially completed python class used to move a robot from any point to a desired point

class GoToPoint:

    def __init__(self):
    
        self.active = False

        # robot state variables
        self.position = Point()
        self.yaw = 0
        # A state machine: 0 - Fix heading; 1 - Go Straight; 2 - Reach the desitination
		#More information can be found at https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
        self.state = 0
        # Destination
        self.desired_position = Point()
        self.desired_position.x = rospy.get_param('des_pos_x')
        self.desired_position.y = rospy.get_param('des_pos_y')
        self.desired_position.z = 0
        # Threshold parameters
        self.yaw_threshold = rospy.get_param('th_yaw') # unit: degree
		self.yaw_threshold *= math.pi/90 # convert to radian
        self.dist_threshold = rospy.get_param('th_dist') # unit: meter

        # publishers
        self.pub_vel = rospy.Publisher('/b00885866Bot/cmd_vel', Twist, queue_size=1)
    
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.callback_odom)
    
        #Declaring a new service named wall_follower_switch with the SetBool service type. 
        #All requests are passed to wall_follower_switch function
        srv = rospy.Service('go_to_point_switch', SetBool, self.go_to_point_switch)
    
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                continue
            else:
                if self.state == 0:
                    self.fix_heading(self.desired_position)
                elif self.state == 1:
                    self.go_straight(self.desired_position)
                elif self.state == 2:
                    self.done()
                else:
                    rospy.logerr('Unknown state!')
        
            rate.sleep()

	# Please complete the rest of code.
	# The overall logic that governs its behavious can be found at 
	# https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
    
if __name__ == '__main__':

    rospy.init_node('go_to_point')
    GoToPoint()
    rospy.spin()
    
    
