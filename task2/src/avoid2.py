#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class ZoneNavigator:
    def __init__(self):
        rospy.init_node('zone_navigator', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.zones = [[-1,-1],[-2,-1],[-2,0],[-2,1],[-1,1],[0,1],[1,1],[1,0],[1,-1],[1,-2],[0,-2],[-1.-2],[-2,-2]]
        self.current_zone_index = 0
        self.obstacle_detected = False
        self.pos_x = 0
        self.pos_y = 0
        self.yaw = 0
        self.front_clear = True
        self.right_clear = True
        self.left_clear = True

    def odom_callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pos_x = position.x
        self.pos_y = position.y
        self.yaw = yaw
        print(f"x={self.pos_x: .2f} [m], y={self.pos_y: .2f} [m], yaw={self.yaw: .1f} [degrees].")

    def scan_callback(self, data):
        front_ranges = data.ranges[:20] + data.ranges[-20:]
        # front_ranges = data.ranges[len(data.ranges) // 3 : 2 * len(data.ranges) // 3]
        # right_ranges = data.ranges[: len(data.ranges) // 3]
        # left_ranges = data.ranges[2 * len(data.ranges) // 3 :]
        
        threshold = 0.5

        self.front_clear = all(r > 0.5 for r in front_ranges)
        # self.right_clear = all(r > 0.3 for r in right_ranges)
        # self.left_clear = all(r > 0.3 for r in left_ranges)

    def move_towards(self, waypoint):
        angle_to_waypoint = math.atan2(waypoint[1] - self.pos_y, waypoint[0] - self.pos_x)
        
        angle_diff = self.normalize_angle(angle_to_waypoint - self.yaw)
        
        speed = Twist()
        if abs(angle_diff) > 0.1: 
            speed.linear.x = 0.0
            speed.angular.z = 0.3 if angle_diff > 0 else -0.3
        else: 
            speed.linear.x = 0.2
            speed.angular.z = 0.0

        self.vel_pub.publish(speed)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def navigate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_zone_index < len(self.zones):
            waypoint = [self.zones[self.current_zone_index][0] + 0.5, self.zones[self.current_zone_index][1] + 0.5]
            
            if abs(self.pos_x - waypoint[0]) < 0.04 and abs(self.pos_y - waypoint[1]) < 0.04: 
            # math.sqrt((self.pos_x - waypoint[0])**2 + (self.pos_y - waypoint[1])**2) < 0.01:
                self.current_zone_index += 1
                print(self.current_zone_index)
                continue
            
            if not self.front_clear:
                stop_msg = Twist()
                self.vel_pub.publish(stop_msg)
                rospy.sleep(1)
                
                # Turn right if right is clear, otherwise turn left
                turn_msg = Twist()
                turn_msg.angular.z = -0.5 
                # if self.right_clear else 0.5
                self.vel_pub.publish(turn_msg)
                rospy.sleep(1)

                continue
                
            self.move_towards(waypoint)

            rate.sleep()

if __name__ == '__main__':
    navigator = ZoneNavigator()
    navigator.navigate()
