#!/usr/bin/env python3

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
import math
import random

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
        occupancy_grid = self.current_map
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        map_data = occupancy_grid.data

        max_unexplored = 0
        best_frontier = None
        
        for frontier in self.frontiers:
            unexplored_count = 0

            for dx in [-3, -2, -1, 0, 1, 2, 3]:
                for dy in [-3, -2, -1, 0, 1, 2, 3]:
                    x = frontier[0] + dx
                    y = frontier[1] + dy
                    i = y * width + x

                    if 0 <= i < len(map_data) and map_data[i] == -1:
                        unexplored_count += 1
            
            if unexplored_count > max_unexplored:
                max_unexplored = unexplored_count
                best_frontier = frontier

        self.closest_frontier = best_frontier
        print(self.closest_frontier)

    def get_random_frontier(self):
        self.closest_frontier = random.choice(list(self.frontiers))
        print(self.closest_frontier)


    def navigate_to_frontier(self, frontier):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        resolution = self.current_map.info.resolution
        origin = self.current_map.info.origin.position
        print(f"Resolution: {resolution}")
        print(f"origin: {origin}")
        goal.pose.position = Point(frontier[0] * resolution + origin.x, frontier[1] * resolution + origin.y, 0)
        goal.pose.orientation.w = 1.0

        goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.sleep(1)
        print("publishing goal")
        goal_publisher.publish(goal)

        
    
    def main(self):
        while self.current_map is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)
        
        while not rospy.is_shutdown():
            self.identify_frontiers()
            print(len(self.frontiers))
            self.get_closest_frontier()
            self.navigate_to_frontier(self.closest_frontier)
            rospy.sleep(10)

        


if __name__ == "__main__":
    explorer = FrontierExploration()
    explorer.main()
    