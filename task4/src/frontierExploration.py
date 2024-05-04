#!/usr/bin/env python3

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
import math

import waffle


from tf.transformations import quaternion_from_euler

from tb3 import Tb3Move

class FrontierExploration:
    def __init__(self):
        rospy.init_node('frontier_exploration', anonymous=False)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        self.motion = waffle.Motion
        self.odom = waffle.Pose(debug=True)

        self.current_map = None
        self.frontiers = None
        self.closest_frontier = None

    def map_callback(self, map_data):
        self.current_map = map_data
        

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
        print(len(self.frontiers))
    
    def get_closest_frontier(self):
        pos_x = self.odom.posx
        pos_y = self.odom.posy

        min_dist = float('inf')
        for frontier in self.frontiers:
            distance = math.sqrt((frontier[0] - pos_x) ** 2 + (frontier[1] - pos_y) ** 2)
            if distance < min_dist:
                min_dist = distance
                self.closest_frontier = frontier
        print(self.closest_frontier)

    def navigate_to_frontier(self, frontier):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position = Point(frontier[0], frontier[1], 0)
        goal.pose.orientation.w = 1.0

        goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.sleep(1)
        print("publishing goal")
        goal_publisher.publish(goal)

        
    
    def main(self):
        while self.current_map is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        self.identify_frontiers()
        self.get_closest_frontier()
        self.navigate_to_frontier(self.closest_frontier)

        


if __name__ == "__main__":
    explorer = FrontierExploration()
    explorer.main()
    