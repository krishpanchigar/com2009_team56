#!/usr/bin/env python3

# TODO: make sure entire width of beacon in in the image
# TODO: add node to launch file

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
import math
import random
import os
from dynamic_reconfigure.client import Client

import waffle

from tf.transformations import quaternion_from_euler

from tb3 import Tb3Move


class FrontierExploration:
    def set_inflation_radius(self, inflation_radius, cost_scaling_factor):
        client = Client("move_base/local_costmap/inflation_layer", timeout=0)
        params = {"inflation_radius": inflation_radius, "cost_scaling_factor": cost_scaling_factor}
        config = client.update_configuration(params)

        client = Client("move_base/global_costmap/inflation_layer", timeout=0)
        params = {"inflation_radius": inflation_radius, "cost_scaling_factor": cost_scaling_factor}
        config = client.update_configuration(params)


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
        self.motion = waffle.Motion()

        self.goal = None
        self.last_position = None
        self.last_time = None
        self.goal_threshold = 0.25

        self.set_inflation_radius(0.1, 1.0)

        #initialize attributes for the camera and color detection
        # TODO: change camera topic for submit
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.target_colour = rospy.get_param('~target_colour', 'green')
        self.m00 = 0
        self.m00_min = 10000
        self.object_position = PoseStamped()
        rospy.loginfo("Target Colour: %s", self.target_colour)

    def map_callback(self, map_data):
        # Check if a new map has been initialized
        if self.current_map is None:
            # Create a new map filled with -1
            self.current_map = OccupancyGrid()
            self.current_map.info = map_data.info
            self.current_map.data = [-1 for _ in range(len(map_data.data))]

        # Get the robot's position in map coordinates
        robot_x = int((self.odom.posx - map_data.info.origin.position.x) / map_data.info.resolution)
        robot_y = int((self.odom.posy - map_data.info.origin.position.y) / map_data.info.resolution)

        # Define the radius of the explored area in map coordinates
        radius = 25  # Adjust this value as needed

        # Copy cells within the radius from the original map to the new map
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:  # If the cell is within the radius
                    x = robot_x + dx
                    y = robot_y + dy
                    if 0 <= x < map_data.info.width and 0 <= y < map_data.info.height:  # If the cell is within the map
                        index = y * map_data.info.width + x
                        self.current_map.data[index] = map_data.data[index]

        # Copy known obstacles from the original map to the new map
        for i, cell in enumerate(map_data.data):
            if cell == 100:  # If the cell is an obstacle
                self.current_map.data[i] = cell

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        height, width, _ = cv_img.shape
        crop_width = width
        crop_height = 300
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))
        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define HSV range for the target color
        if self.target_colour == "green":
            lower = (81, 175, 0)
            upper = (92, 255, 255)
        elif self.target_colour == "red":
            lower = (0, 175, 0)
            upper = (7, 255, 255)
        elif self.target_colour == "yellow":
            lower = (23, 175, 0)
            upper = (28, 250, 255)
        elif self.target_colour == "blue":
            lower = (115, 224, 100)
            upper = (130, 255, 255)

        mask = cv2.inRange(hsv_img, lower, upper)
        m = cv2.moments(mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(crop_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    focal_length = 1206.8897719532354
                    real_object_width = 200
                    distance_mm = (focal_length * real_object_width) / w
                    distance_m = distance_mm / 1000 + 0.1
                    beacon_width_pixels = (distance_mm + w) / focal_length
                    print(f"Beacon width: {beacon_width_pixels}")
                    print(f"W: {w}")

                    snaps_dir = os.path.join(os.path.expanduser('~'), 'catkin_ws/src/com2009_team56/snaps')
                    if not os.path.exists(snaps_dir):
                        os.makedirs(snaps_dir)
                    filepath = os.path.join(snaps_dir, "task4_beacon.jpg")
                    print(f"H/W {h/w}")

                    if h/w >= 1.24 and h/w <= 1.26:
                        self.motion.stop()
                        cv2.imwrite(filepath, crop_img)
                        self.camera_subscriber.unregister()

                    # Assume the robot yaw and global position are known
                    object_x = distance_m * math.cos(0.0)  # Replace with your robot's yaw
                    object_y = distance_m * math.sin(0.0)

                    # Store the detected object's global position
                    self.object_position.header.frame_id = "map"
                    self.object_position.header.stamp = rospy.Time.now()
                    self.object_position.pose.position.x = object_x
                    self.object_position.pose.position.y = object_y
                    self.object_position.pose.orientation.w = 1.0

                    self.target_detected = True

        # Display the image
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

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
                    wall_neighbour = False
                    for dx in [-2, -1, 0, 1, 2]:
                        for dy in [-2, -1, 0, 1, 2]:
                            ni = (y + dy) * width + (x + dx)

                            if (0 <= ni < len(map_data)):
                                if map_data[ni] == -1:
                                    wall_neighbour = True
                                elif map_data[ni] == 100:
                                    wall_neighbour = True
                                    break

                        if wall_neighbour:
                            break

                    if not wall_neighbour:
                        self.frontiers.add((x, y))
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

    def get_random_frontier(self):
        self.closest_frontier = random.choice(list(self.frontiers))
        print(self.closest_frontier)

    def navigate_to_frontier(self, frontier):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        resolution = self.current_map.info.resolution
        origin = self.current_map.info.origin.position
        self.goal = (frontier[0] * resolution + origin.x, frontier[1] * resolution + origin.y)
        goal.pose.position = Point(frontier[0] * resolution + origin.x, frontier[1] * resolution + origin.y, 0)
        goal.pose.orientation.w = 1.0

        goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.sleep(1)
        print("publishing goal")
        goal_publisher.publish(goal)

    def check_preempt_conditions(self):
        # Preempt condition 1: If robot's x and y are the same as the goal's x and y
        distance_to_goal = math.sqrt((self.odom.posx - self.goal[0]) ** 2 + (self.odom.posy - self.goal[1]) ** 2)
        if distance_to_goal <= self.goal_threshold:
            rospy.loginfo("Preempting goal due to reaching the destination.")
            self.client.cancel_all_goals()
            self.identify_frontiers()
            self.get_closest_frontier()
            self.navigate_to_frontier(self.closest_frontier)
            return True

        # Preempt condition 2: If robot is stationary for 5 seconds
        current_time = rospy.Time.now()
        if self.last_position == (self.odom.posx, self.odom.posy) and (current_time - self.last_time).to_sec() > 5:
            rospy.loginfo("Preempting goal due to robot being stationary for 5 seconds.")
            self.client.cancel_all_goals()
            self.identify_frontiers()
            self.get_random_frontier()
            self.navigate_to_frontier(self.closest_frontier)
            return True

        return False

    def main(self):
        while self.current_map is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        self.last_position = (self.odom.posx, self.odom.posy)
        self.last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            self.identify_frontiers()
            print(len(self.frontiers))
            self.get_closest_frontier()
            self.navigate_to_frontier(self.closest_frontier)
            print(f"Current: {self.odom.posx, self.odom.posy}. Goal: {self.goal}")
            while True:
                current_time = rospy.Time.now()
                if self.check_preempt_conditions():
                    self.last_position = (self.odom.posx, self.odom.posy)
                    self.last_time = rospy.Time.now()

                elapsed_time = current_time - start_time
                if elapsed_time.to_sec() >= 20:
                    break
            rospy.sleep(0.1)
            # Update last_position and last_time at the end of each loop
            self.last_position = (self.odom.posx, self.odom.posy)
            self.last_time = rospy.Time.now()


            


if __name__ == "__main__":
    explorer = FrontierExploration()
    explorer.main()