#!/usr/bin/env python
# The code was adapted from https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot
#  Or from lecture handouts

import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleDetectionAndAvoidance():
    # Class that attempts to maintain a fixed distance from the obstacle ahead. 
    def __init__(self):
        # Set up the node, publishers and subscribers.
        rospy.init_node('Obstacle_detection_min_distance')
        self.scan_sub = rospy.Subscriber('/group19Bot/laser/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/group19Bot/cmd_vel', Twist, queue_size=2) 
        self.target = .75
        self.range_ahead = 1

    def scan_callback(self, scan_msg):
    
        self.range_ahead = min(scan_msg.ranges)
        # Move forward or backward depending on the latest scan.
        cmd_vel = Twist()
        
        # Back up if the scan is bad, or if we are too close. 
        if(math.isnan(self.range_ahead) or self.range_ahead < self.target):
            cmd_vel.linear.x = -1.0
            cmd_vel.angular.z = 1.0   
        else:
            cmd_vel.linear.x = 1
            cmd_vel.angular.z = 0   
        
        rospy.loginfo("Range: {:.3f}".format(self.range_ahead))
        self.cmd_pub.publish(cmd_vel) 

    def run(self):
        # Now enter an infinite loop. Execution will be driven by the
        # callback.
        rospy.spin() 
        
if __name__ == "__main__":
    try:
        oda = ObstacleDetectionAndAvoidance()
        oda.run()
    except rospy.ROSInterruptException:
        pass
