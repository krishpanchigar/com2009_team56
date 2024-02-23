#!/usr/bin/env python3
# TODO: make it turn and stop based on coordinates maybe


import rospy
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import time

class MoveEight:
    def __init__(self):
        rospy.init_node('circle_motion_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.rate = rospy.Rate(10)  
        self.start_position = None
        self.start_yaw = None
        self.current_position = None
        self.current_yaw = None
        self.last_position = None
        self.last_yaw = None
        self.total_distance = 0
        self.first_circle = True
        self.second_circle = False
    
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        if self.start_position is None:
            self.start_position = position
            self.current_position = position
        
        self.current_position.x = position.x - self.start_position.x
        self.current_position.y = position.y - self.start_position.y
        self.current_position.z = position.z - self.start_position.z

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                        orientation.z, orientation.w], "sxyz")
        
        yaw_degrees = yaw * 180 / math.pi

        if self.first_circle and self.current_position.x > 0.4:
            self.first_circle = False

        if not self.first_circle and self.current_position.x < -0.4:
            self.second_circle = True

        if self.start_yaw is None:
            self.start_yaw = yaw * 180 / math.pi

        self.current_yaw = yaw_degrees - self.start_yaw  

        pos_x = position.x
        pos_y = position.y

        print(f"x={pos_x: .2f} [m], y={pos_y: .2f} [m], yaw={self.current_yaw: .1f} [degrees].")

    def move_eight(self, clockwise = False):
        twist = Twist()
        twist.linear.x = math.pi / 30
    
        while not rospy.is_shutdown():
            if self.first_circle:
                twist.angular.z = -math.pi / 15
            elif not self.first_circle and not self.second_circle and self.current_position.x <= 0.05 and self.current_position.y <= 0.05 and self.current_yaw <= 1:
                twist.angular.z = math.pi / 15
            elif not self.first_circle and self.second_circle and self.current_position.x <= 0.05 and self.current_position.y <= 0.05 and self.current_yaw <= 1:
                break

            self.pub.publish(twist)
            self.rate.sleep()

    def stop_robot(self):
        twist = Twist()  
        self.pub.publish(twist)

    def main(self):
        self.move_eight(clockwise=False)    
        self.stop_robot() 
        rospy.loginfo("Robot stopped.")


if __name__ == '__main__':
    move_eight = MoveEight()
    move_eight.main()