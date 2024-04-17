#!/usr/bin/env python3

import rospy
import tf
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

class RobotSLAM:
    def __init__(self):
        rospy.init_node('robot_slam', anonymous=False)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    def set_initial_pose(self, x, y, theta):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))
        
        self.initial_pose_pub.publish(initial_pose)
        rospy.sleep(1)

    def move_to_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def save_map(self, map_name):
        map_saver_command = "rosrun map_server map_saver -f " + map_name
        os.system(map_saver_command)

# Main execution
if __name__ == '__main__':
    robot_slam = RobotSLAM()

    robot_slam.set_initial_pose(0, 0, 0)

    waypoints = [(1, 1), (2, 2), (3, 1)] 

    for point in waypoints:
        result = robot_slam.move_to_goal(point[0], point[1])
        if result:
            rospy.loginfo("Reached waypoint %s", point)
        else:
            rospy.loginfo("Failed to reach waypoint %s", point)

    # Save the map at the end of exploration
    robot_slam.save_map("my_robot_map")

    rospy.loginfo("Exploration and map saving complete.")
    rospy.signal_shutdown("Exploration and map saving complete.")
