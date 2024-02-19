#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import time
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class MoveEight:
    def __init__(self):
        rospy.init_node('circle_motion_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.rate = rospy.Rate(10)  
        self.start_time = None

    
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        pos_x = position.x
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                        orientation.z, orientation.w], "sxyz")
        yaw_degrees = yaw * 180 / math.pi

        print(f"x={pos_x} [m], y={pos_y} [m], yaw={yaw_degrees} [degrees].")

    def move_eight(self, clockwise=False):
        twist = Twist()
        twist.linear.x = math.pi / 30  
        if clockwise:
            twist.angular.z = -math.pi/15  
        else:
            twist.angular.z = math.pi/15

        
        if self.start_time is None:
            self.start_time = time.time()

        
        duration = 31
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