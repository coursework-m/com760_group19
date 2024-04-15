#!/usr/bin/env python

## Simple publisher demo that published geometry_msgs/Twist messages
## to the /turtle1/cmd_vel topic

import rospy
from geometry_msgs.msg import Twist

def move_circle():
    #Initialize a publisher that can talk to the turtlesim and ask it to move
	pub = rospy.Publisher('week7bot/cmd_vel', Twist, queue_size=10)
	rospy.init_node('turtle_mover')

    #create a Twist message and linear x and angular z value
	vel_msg = Twist()
	vel_msg.linear.x = 1.0
	vel_msg.angular.z = 1.0
    
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		rospy.loginfo(vel_msg)
		pub.publish(vel_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		move_circle()
	except rospy.ROSInterruptException:
		pass
