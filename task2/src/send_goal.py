#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from move_base_msgs.msg import MoveBaseActionResult

import time

class SendGoal:
    def __init__(self):
        rospy.init_node('multi_point_navigator', anonymous=True)

        self.last_print_time = rospy.Time.now() 
        self.start_position = None
        self.is_bot_ready = True
        self.rate = rospy.Rate(10)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.sub_goal_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_cb)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.sleep(1) 

        self.waypoints = [
            (-1.5, -0.5, math.pi / 2),
            (-1.5, 0.5, math.pi / 2),
            (-1.5, 1.5, math.pi),
            (-0.5, 1.5, math.pi),
            (0.5, 1.5, math.pi),
            (1.5, 1.5, -math.pi / 2),
            (1.5, 0.5, -math.pi / 2),
            (1.5, -0.5, -math.pi / 2),
            (1.5, -1.5, -math.pi),
            (0.5, -1.5, -math.pi),
            (-0.5, -1.5, -math.pi),
            (-1.5, -1.5, math.pi)
        ]                
        

    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                        orientation.z, orientation.w], "sxyz")
        pos_x = position.x
        pos_y = position.y

        current_time = rospy.Time.now()
        if (current_time - self.last_print_time).to_sec() >= 1.0:
            print(f"x={pos_x: .2f} [m], y={pos_y: .2f} [m], yaw={yaw: .1f} [degrees].")
            self.last_print_time = current_time

    

    
    def send_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = z
        self.pub_goal.publish(goal)
        self.is_bot_ready = False
        rospy.loginfo(f"Sending goal to ({x}, {y}, {z})")

    def navigate(self):
        for waypoint in self.waypoints:
            x, y, z = waypoint
            if self.is_bot_ready:
                self.send_goal(x, y, z)

if __name__ == "__main__":
    navigator = SendGoal()
    navigator.navigate()
