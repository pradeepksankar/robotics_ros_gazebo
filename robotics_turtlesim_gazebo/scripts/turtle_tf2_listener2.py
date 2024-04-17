#!/usr/bin/env python
import random
import rospy
import tf
import math
import tf2_ros
from geometry_msgs.msg import *
from turtlesim.msg import *
from turtlesim.srv import *

if __name__ == '__main__':
	rospy.init_node('turtle_tf2_listener2')

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rospy.wait_for_service('spawn')
	spawner = rospy.ServiceProxy('spawn', Spawn)
	turtle_name = rospy.get_param('turtle', 'followerB')
	followerB_x = random.randint(0, 10)
	followerB_y = random.randint(0, 10)
	followerB_z = random.randint(0, 10)
	spawner(followerB_x, followerB_y, followerB_z, turtle_name)

	turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, Twist, queue_size=1)

	rospy.wait_for_service("/followerB/set_pen")
	setPen = rospy.ServiceProxy("/followerB/set_pen", SetPen)
	setPen(255, 255, 255, 3, 0)
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		try:
			br.sendTransform((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "carrot2", "followerA")
			trans = tfBuffer.lookup_transform(turtle_name, 'carrot2', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue

		msg = geometry_msgs.msg.Twist()

		msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
		msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

		turtle_vel.publish(msg)

		rate.sleep()
