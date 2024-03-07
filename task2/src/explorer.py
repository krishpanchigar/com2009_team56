#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Explorer:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
        self.twist = Twist()

    def callback(self, data):
        # If there's an obstacle within 1 meter, turn left
        if min(data.ranges) < 0.25:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        # Otherwise, move forward
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
        self.pub.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('explorer')
    explorer = Explorer()
    rospy.spin()