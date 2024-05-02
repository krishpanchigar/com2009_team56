#!/usr/bin/env python3

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid

from tf.transformations import quaternion_from_euler

from tb3 import Tb3Move

class FrontierExploration:
    def __init__(self):
        rospy.init_node('frontier_exploration', anonymous=False)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        self.robot_controller = Tb3Move()

        self.current_map = None

    def map_callback(self, map_data):
        self.current_map = map_data
        print(map_data.info)
        print(map_data.data)

    def identify_frontiers(self, map_data):
        # Identify frontiers in the map
        pass