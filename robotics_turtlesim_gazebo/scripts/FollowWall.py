#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations


# A partially completed python class used to make the robot follow a wall

class FollowWall:
    
    def __init__(self):
        
		self.active = False
       
		#A state machine: 0 - Find the wall; 1 - Turn left; 2 - Follow the wall
		#More information can be found at https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
        
        self.state = 0
        
        self.pub_vel = rospy.Publisher('/com760spBot/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber('/com760sp/laser/scan', LaserScan, self.callback_laser)
    
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue
        
            msg = Twist()
            if self.state == 0:
                msg = self.find_wall()
            elif self.state == 1:
                msg = self.turn_left()
            elif self.state == 2:
                msg = self.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
            
            self.pub_vel.publish(msg)
            
            rate.sleep()

	# Please complete the rest of code.
	# The overall logic that governs its behavious can be found at 
	# https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

if __name__=='__main__':
    
    rospy.init_node('follow_wall')
    FollowWall()
    rospy.spin()


