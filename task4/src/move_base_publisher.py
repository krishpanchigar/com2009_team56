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
        self.increment = 0
        self.goal_threshold = 0.5
        self.current_goal = None
        self.last_move_time = rospy.Time.now()
        self.last_position = None
        self.goal_start_time = None

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.sleep(1)

        self.waypoints = [
            (0, 0),
            (-1.42, 0),
            (-1.42, -1.42),
            (-1.42, 1.42),
            (0, -1.42),
            (0, 1.42),
            (1.42, 0),
            (1.42, -1.42),
            (1.42, 1.42)
        ]

    def send_goal(self, x, y, z, w):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.pub_goal.publish(goal)
        self.is_bot_ready = False
        self.goal_start_time = rospy.Time.now()
        rospy.loginfo(f"Sending goal to ({x}, {y})")

    def navigate(self):
        for waypoint in self.waypoints:
            x, y, z, w = waypoint
            while not self.is_bot_ready:
                # if the robot is stationary for more than 2 seconds, move to next goal
                if (rospy.Time.now() - self.last_move_time).to_sec() > 2:
                    rospy.loginfo("Bot is stationary for more than 2 seconds. Preempting goal.")
                    self.is_bot_ready = True
                elif self.goal_start_time is not None and (rospy.Time.now() - self.goal_start_time).to_sec() > 9:
                    rospy.loginfo("Goal timed-out.")
                    self.is_bot_ready = True
                rospy.sleep(0.1)
            self.send_goal(x, y, z, w)
            

if __name__ == "__main__":
    navigator = SendGoal()
    navigator.navigate()
