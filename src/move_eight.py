#!/usr/bin/env python3
# TODO: make it turn and stop based on coordinates maybe


import rospy
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class MoveEight:
    def __init__(self):
        rospy.init_node('circle_motion_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.rate = rospy.Rate(10)  
        self.start_position = None
        self.start_yaw = None
        self.current_position = None
        self.last_position = None
        self.total_distance = 0
    
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        if self.start_position is None:
            self.start_position = position
            self.last_position = position
        
        if self.start_yaw is None:
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                        orientation.z, orientation.w], "sxyz")
            self.start_yaw = yaw * 180 / math.pi

        self.current_position = position 
        distance = math.sqrt((self.current_position.x - self.last_position.x) ** 2 +
                                 (self.current_position.y - self.last_position.y) ** 2)
        self.total_distance += distance
        self.last_position = self.current_position  

        pos_x = position.x - self.start_position.x
        pos_y = position.y - self.start_position.y

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                        orientation.z, orientation.w], "sxyz")
        yaw_degrees = yaw * 180 / math.pi
        yaw_degrees_adjusted = yaw_degrees - self.start_yaw

        print(f"x={pos_x: .2f} [m], y={pos_y: .2f} [m], yaw={yaw_degrees_adjusted: .2f} [degrees].")

    def move_eight(self, clockwise = False):
        twist = Twist()
        twist.linear.x = math.pi / 30

        while not rospy.is_shutdown():
            if self.total_distance <= 0.99*math.pi:
                twist.angular.z = math.pi/15 if not clockwise else -math.pi/15
            else:
                twist.angular.z = -math.pi/15 if not clockwise else math.pi/15

            if self.total_distance > 2 * math.pi:
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