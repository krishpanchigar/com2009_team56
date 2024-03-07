#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from map_server.srv import SaveMap

class MapUpdater:
    def __init__(self):
        self.checkpoints = [[-1,-1],[-2,-1],[-2,0],[-2,1],[-1,1],[0,1],[1,1],[1,0],[1,-1],[1,-2],[0,-2],[-1.-2],[-2,-2]]
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def save_map(self):
        rospy.wait_for_service('static_map')
        save_map_service = rospy.ServiceProxy('static_map', SaveMap)
        try:
            save_map_service("/path/to/your/map")
            rospy.loginfo("Map saved.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def send_goal(self, checkpoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = checkpoint[0] + 0.5
        goal.target_pose.pose.position.y = checkpoint[1] + 0.5
        goal.target_pose.pose.orientation.w = 1.0  # Assuming all checkpoints are oriented towards the positive x-axis
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def update_map(self):
        for checkpoint in self.checkpoints:
            self.send_goal(checkpoint)
            self.save_map()

if __name__ == '__main__':
    rospy.init_node('map_updater')
    map_updater = MapUpdater()
    map_updater.update_map()