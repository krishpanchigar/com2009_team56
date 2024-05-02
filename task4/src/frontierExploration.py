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
        self.frontiers = None

    def map_callback(self, map_data):
        self.current_map = map_data
        self.identify_frontiers()
        print(len(self.frontiers))
        

    def identify_frontiers(self):
        # Identify frontiers in the map
        occupancy_grid = self.current_map
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        map_data = occupancy_grid.data

        self.frontiers = set()

        # iterating over the map data
        for y in range(height):
            for x in range(width):
                # index of cell
                i = y * width + x

                if map_data[i] == 0:
                    # check for neighbouring cells
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            ni = (y + dy) * width + (x + dx)

                            if(0 <= ni < len(map_data) and map_data[ni] == -1):
                                self.frontiers.add((x, y))
                                break
    
    def main(self):
        self.identify_frontiers()

        


if __name__ == "__main__":
    explorer = FrontierExploration()
    explorer.main()
    