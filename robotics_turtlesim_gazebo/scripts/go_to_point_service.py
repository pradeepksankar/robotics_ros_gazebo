#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
#desired_position_.x = -3
#desired_position_.y = 7
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callback function for the subscriber defined in main
#the function receives the odometry data and extracs the position and yaw information
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    # Bear in mind the odometry data encodes orientation information in quaternions
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    #To obtains yaw the quaternion is converted into euler angles
    euler = transformations.euler_from_quaternion(quaternion) 
    yaw_ = euler[2]

#This function changes the value of the global state variable that stores the robot state information
def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

#This function is executed when robot is in state 0 (Fix heading). First the current heading of the robot is checked with desired heading. 
#If the differene in heading is more than a threshold the robot is commanded to turn in its place.
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

#This function is executed when robot is in state 1 (Go Straight). This state occurs after robot has fixed the error in yaw. 
#In this state, the distance between the robots current position and desired position is compared with a threshold. 
#If robot is further away from desired position it is commanded to move forward. 
#If the current position lies closer to the desired position then the yaw is again checked for error, 
#if yaw is significantly different from the desired yaw value the robot goes to state 0.

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)
        
#Eventually robot achieves correct heading and correct position. Once in this state the robot stops.
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():#entry point
    global pub, active_
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/b00885866Bot/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    main()
