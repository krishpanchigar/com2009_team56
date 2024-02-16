#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import time

class MoveEight:
    def __init__(self):
        rospy.init_node('circle_motion_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  
        self.start_time = None

    def move_eight(self, clockwise=False):
        twist = Twist()
        twist.linear.x = math.pi / 30  
        if clockwise:
            twist.angular.z = -math.pi/15  
        else:
            twist.angular.z = math.pi/15

        
        if self.start_time is None:
            self.start_time = time.time()

        
        duration = 32  
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(twist)
            self.rate.sleep()

    def stop_robot(self):
        twist = Twist()  
        self.pub.publish(twist)

    def main(self):
        self.move_eight(clockwise=False)  
        self.move_eight(clockwise=True)  
        self.stop_robot() 
        rospy.loginfo("Robot stopped.")

        total_duration = time.time() - self.start_time
        rospy.loginfo(f"Total duration: {total_duration} seconds.")

        if math.isclose(total_duration, 60, abs_tol=2):
            rospy.loginfo("Total duration is within the acceptable range.")
        else:
            rospy.logwarn("Total duration deviates significantly from the expected value.")

if __name__ == '__main__':
    move_eight = MoveEight()
    move_eight.main()