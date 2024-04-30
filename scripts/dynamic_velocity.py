#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DynamicVelocity():
    def __init__(self):
        rospy.init_node('Obstacle_avoidance')
        self.pub = rospy.Publisher('/group19Bot/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/group19Bot/laser/scan', LaserScan, self.clbk_laser)

        # Define parameters for velocity scaling
        self.max_linear_velocity = 0.7  # Maximum linear velocity
        self.max_angular_velocity = 0.4  # Maximum angular velocity
        self.min_distance = 0.5  # Minimum distance to obstacles for full speed
        self.max_distance = 2.0  # Maximum distance to obstacles for no speed

    def clbk_laser(self, msg):
        regions = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'front_right': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'front_left':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }
        self.take_action(regions)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        angular_z = 0
        
        state_description = ''

        # Determine linear velocity based on distance to obstacles
        linear_x = self.max_linear_velocity * (regions['front'] - self.min_distance) / (self.max_distance - self.min_distance)
        linear_x = min(max(linear_x, 0), self.max_linear_velocity)  # Clip velocity to [0, max_linear_velocity]

        # Determine angular velocity based on the difference between front and side distances
        if regions['front_left'] > regions['front_right']:
            angular_z = -self.max_angular_velocity
        else:
            angular_z = self.max_angular_velocity

        # Update state description for logging
        state_description = {'linear_x': linear_x, 'angular_z': angular_z}

        rospy.loginfo(state_description)

        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        avoidance = DynamicVelocity()
        avoidance.run()
    except rospy.ROSInterruptException:
        pass