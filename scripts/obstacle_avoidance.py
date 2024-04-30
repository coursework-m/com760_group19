#! /usr/bin/env python
# The code was adapted from https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot
#  Or from lecture handouts

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance():
    def __init__(self):
        rospy.init_node('Obstacle_avoidance')
        self.scan_sub = rospy.Subscriber('/group19Bot/laser/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/group19Bot/cmd_vel', Twist, queue_size=1)
        

    def scan_callback(self, cmd_vel):
        regions = {
            'right':  min(min(cmd_vel.ranges[0:143]), 10),
            'front_right': min(min(cmd_vel.ranges[144:287]), 10),
            'front':  min(min(cmd_vel.ranges[288:431]), 10),
            'front_left':  min(min(cmd_vel.ranges[432:575]), 10),
            'left':   min(min(cmd_vel.ranges[576:719]), 10),
        }
        self.take_action(regions)

    def take_action(self, regions):
        cmd_vel = Twist()
        linear_x = 0
        angular_z = 0
        
        state_description = ''
    
        if regions['front'] > 1 and regions['front_left'] > 1 and regions['front_right'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0
        elif regions['front'] < 1 and regions['front_left'] > 1 and regions['front_right'] > 1:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] > 1 and regions['front_left'] > 1 and regions['front_right'] < 1:
            state_description = 'case 3 - front_right'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] > 1 and regions['front_left'] < 1 and regions['front_right'] > 1:
            state_description = 'case 4 - front_left'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] < 1 and regions['front_left'] > 1 and regions['front_right'] < 1:
            state_description = 'case 5 - front and front_right'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['front_left'] < 1 and regions['front_right'] > 1:
            state_description = 'case 6 - front and front_left'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] < 1 and regions['front_left'] < 1 and regions['front_right'] < 1:
            state_description = 'case 7 - front and front_left and front_right'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] > 1 and regions['front_left'] < 1 and regions['front_right'] < 1:
            state_description = 'case 8 - front_left and front_right'
            linear_x = 0.3
            angular_z = 0
        else:
            state_description = 'unknown case'
        
        rospy.loginfo(regions)
        rospy.loginfo(state_description)

        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        avoidance = ObstacleAvoidance()
        avoidance.run()
    except rospy.ROSInterruptException:
        pass

