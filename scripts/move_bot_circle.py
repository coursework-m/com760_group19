#!/usr/bin/env python
# This code was adapted from a lecture handout and ros tutorial

import rospy
from geometry_msgs.msg import Twist

class CircleMover():
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('Circle_mover')
        
        # Initialize a publisher to control the robot's movement
        self.vel_pub = rospy.Publisher('/group19Bot/cmd_vel', Twist, queue_size=2)
        
        # Create a Twist message to send velocities
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.5 # Linear velocity along the x-axis
        self.cmd_vel.angular.z = 0.3  # Angular velocity around the z-axis
        
        # Set the publishing rate
        self.rate = rospy.Rate(20)  # 20 Hz
    
    def move(self):
        # Loop until ROS is shutdown
        while not rospy.is_shutdown():
            # Publish the Twist message
            self.vel_pub.publish(self.cmd_vel)
            
            # Log the Twist message
            rospy.loginfo(self.cmd_vel)
            
            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mover = CircleMover()
        mover.move()
    except rospy.ROSInterruptException:
        pass
