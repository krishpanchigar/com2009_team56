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
        self.sub_goal_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_cb)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.sleep(1)
        
        self.increment += 1
        if self.increment == 10:
            self.is_bot_ready = True
            rospy.loginfo("Goal timed-out.")
            self.increment = 0

        self.waypoints = [
            (-1.5, -0.5, 0.7, 0.7),
            (-1.5, 0.5, 0.7, 0.7),
            (-1.5, 1.5, 0, -1),
            (-0.5, 1.5, 0, -1),
            (0.5, 1.5, 0, -1),
            (1.5, 1.5, -0.7, 0.7),
            (1.5, 0.5, -0.7, 0.7),
            (1.5, -0.5, -0.7, 0.7),
            (1.5, -1.5, -1, 0),
            (0.5, -1.5, -1, 0),
            (-0.5, -1.5, -1, 0),
            (-1.5, -1.5, -1, 0)
        ]                

    def goal_result_cb(self, data):
        if data.status.status == 3:
            self.is_bot_ready = True
            rospy.loginfo("Goal reached.")


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

        # Update last_move_time
        if self.last_position is None or (pos_x != self.last_position[0] and pos_y != self.last_position[1]):
            self.last_move_time = rospy.Time.now()
            self.last_position = (pos_x, pos_y)

        # Preempt the goal if the bot is in the vicinity of the goal
        if self.current_goal is not None:
            distance_to_goal = math.sqrt((pos_x - self.current_goal[0])**2 + (pos_y - self.current_goal[1])**2)
            if distance_to_goal < self.goal_threshold:
                self.is_bot_ready = True
                rospy.loginfo("Goal preempted.")
                self.current_goal = None

    
    def send_goal(self, x, y, z, w):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        self.current_goal = (x, y)
        self.pub_goal.publish(goal)
        self.is_bot_ready = False
        self.goal_start_time = rospy.Time.now()
        rospy.loginfo(f"Sending goal to ({x}, {y}, {z}, {w})")

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
