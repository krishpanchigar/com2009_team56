#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class Avoidance:
    def __init__(self):
        rospy.init_node('avoidance_node', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.rate = rospy.Rate(10)
        self.start_position = None
        self.start_yaw = None
        self.current_position = None
        self.current_yaw = None
        self.last_position = None
        self.last_yaw = None
        self.total_distance = 0
        self.last_print_time = rospy.Time.now() 
    
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
        
        yaw_degrees = math.degrees(yaw) if math.degrees(yaw) >= 0 else 360 + math.degrees(yaw)

        if self.start_yaw is None:
            self.start_yaw = math.degrees(yaw) if math.degrees(yaw) >= 0 else 360 + math.degrees(yaw)

        self.current_yaw = yaw_degrees - self.start_yaw  
        if self.current_yaw < 0:
            self.current_yaw += 360

        pos_x = position.x
        pos_y = position.y

        current_time = rospy.Time.now()
        if (current_time - self.last_print_time).to_sec() >= 1.0:
            print(f"x={pos_x: .2f} [m], y={pos_y: .2f} [m], yaw={self.current_yaw: .1f} [degrees].")
            self.last_print_time = current_time
            
    def lidar_callback(self, data):
        front_ranges = data.ranges[:20] + data.ranges[-20:]
        left_range = data.ranges[:80] + data.ranges[100:]
        right_range = data.ranges[:260] + data.ranges[280:]
        min_front_range = min(front_ranges)
        min_left_range = min(left_range)
        min_right_range = min(right_range)
        twist = Twist()

        if min_right_range < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 1
        else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
        
        if min_left_range < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 1
        else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0

        if min_front_range < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 1
        else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0

        self.velocity_publisher.publish(twist)
    
    def run(self):
            rospy.spin()

if __name__ == '__main__':
    avoidance = Avoidance()
    avoidance.run()
