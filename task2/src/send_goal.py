#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class SendGoal:
    def __init__(self):
        rospy.init_node('multi_point_navigator', anonymous=True)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.sleep(1) 
        self.waypoints = [
            (-0.5, -0.5),
            (-1.5, 0.5),
            (-1.5, 1.5),
            (-0.5,1.5),
            (0.5, 1.5),
            (1.5, 1.5),
            (1.5, 0.5),
            (1.5, -0.5),
            (1.5, -1.5),
            (0.5, -1.5),
            (-0.5, -1.5),
            (-1.5, -1.5)
        ]

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)
        rospy.loginfo(f"Sending goal to ({x}, {y})")

    def navigate(self):
        for waypoint in self.waypoints:
            x, y = waypoint
            self.send_goal(x, y)
            rospy.sleep(10)  

if __name__ == "__main__":
    navigator = SendGoal()
    navigator.navigate()
