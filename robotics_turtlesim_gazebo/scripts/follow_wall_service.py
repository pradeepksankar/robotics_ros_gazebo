#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None
# this is a dictionary variable that holds the distances to five directions
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


##The function is called with instances of SetBoolRequest and returns instances of SetBoolResponse.
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res
    
#This function is passed to the Subscriber method and it executes when a new laser data is made available. 
#This function writes distance values in the global variable regions_ and calls the function take_actions
#convert the 720 readings contained inside the LaserScan msg into five distinct readings
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

#This function manipulates the state of the robot. The various distances stored in regions_ variable help in determining the state of the robot.
#Based on the distance sensed in the 3 region (front-left, front, front-right) to determine the state of the robot

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

#This function defines the action to be taken by the robot when it is not surrounded by any obstacle. 
#This method essentially makes the robot move in a anti-clockwise circle (until it finds a wall).
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

#Once the robot is positioned such that its front and front-left path is clear 
#while its front-right is obstructed the robot goes into the follow wall state. 
#In this state this function is executed and this function makes the robot to follow a straight line.
def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/b00885866Bot/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/b00885866Bot/laser/scan', LaserScan, clbk_laser)
    
    #Declaring a new service named wall_follower_switch with the SetBool service type. 
    #All requests are passed to wall_follower_switch function.
     
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
